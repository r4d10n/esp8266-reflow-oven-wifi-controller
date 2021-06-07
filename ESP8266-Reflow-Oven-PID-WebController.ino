/*******************************************************************************
* Title: Reflow Oven Controller
* Version: 1.20
* Date: 26-11-2012
* Company: Rocket Scream Electronics
* Author: Lim Phang Moh
* Website: www.rocketscream.com
* 
* Brief
* =====
* This is an example firmware for our Arduino compatible reflow oven controller. 
* The reflow curve used in this firmware is meant for lead-free profile 
* (it's even easier for leaded process!). You'll need to use the MAX31855 
* library for Arduino if you are having a shield of v1.60 & above which can be 
* downloaded from our GitHub repository. Please check our wiki 
* (www.rocketscream.com/wiki) for more information on using this piece of code 
* together with the reflow oven controller shield. 
*
* Temperature (Degree Celcius)                 Magic Happens Here!
* 245-|                                               x  x  
*     |                                            x        x
*     |                                         x              x
*     |                                      x                    x
* 200-|                                   x                          x
*     |                              x    |                          |   x   
*     |                         x         |                          |       x
*     |                    x              |                          |
* 150-|               x                   |                          |
*     |             x |                   |                          |
*     |           x   |                   |                          | 
*     |         x     |                   |                          | 
*     |       x       |                   |                          | 
*     |     x         |                   |                          |
*     |   x           |                   |                          |
* 30 -| x             |                   |                          |
*     |<  60 - 90 s  >|<    90 - 120 s   >|<       90 - 120 s       >|
*     | Preheat Stage |   Soaking Stage   |       Reflow Stage       | Cool
*  0  |_ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _|_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ 
*                                                                Time (Seconds)
*
* This firmware owed very much on the works of other talented individuals as
* follows:
* ==========================================
* Brett Beauregard (www.brettbeauregard.com)
* ==========================================
* Author of Arduino PID library. On top of providing industry standard PID 
* implementation, he gave a lot of help in making this reflow oven controller 
* possible using his awesome library.
*
* ==========================================
* Limor Fried of Adafruit (www.adafruit.com)
* ==========================================
* Author of Arduino MAX6675 library. Adafruit has been the source of tonnes of
* tutorials, examples, and libraries for everyone to learn.
*
* Disclaimer
* ==========
* Dealing with high voltage is a very dangerous act! Please make sure you know
* what you are dealing with and have proper knowledge before hand. Your use of 
* any information or materials on this reflow oven controller is entirely at 
* your own risk, for which we shall not be liable. 
*
* Licences
* ========
* This reflow oven controller hardware and firmware are released under the 
* Creative Commons Share Alike v3.0 license
* http://creativecommons.org/licenses/by-sa/3.0/ 
* You are free to take this piece of code, use it and modify it. 
* All we ask is attribution including the supporting libraries used in this 
* firmware. 
*
* Required Libraries
* ==================
* - Arduino PID Library: 
*   >> https://github.com/br3ttb/Arduino-PID-Library
* - MAX31855 Library (for board v1.60 & above): 
*   >> https://github.com/rocketscream/MAX31855
* - MAX6675 Library (for board v1.50 & below):
*   >> https://github.com/adafruit/MAX6675-library
*
* Revision  Description
* ========  ===========
* 1.20      Adds supports for v1.60 (and above) of Reflow Oven Controller 
*           Shield:
*           - Uses MAX31855KASA+ chip and pin reassign (allowing A4 & A5 (I2C)
*             to be used for user application).
*           - Uses analog based switch (allowing D2 & D3 to be used for user 
*             application). 
*           Adds waiting state when temperature too hot to start reflow process.
*           Corrected thermocouple disconnect error interpretation (MAX6675).
* 1.10      Arduino IDE 1.0 compatible.
* 1.00      Initial public release.
*******************************************************************************/
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <PID_v1.h>

const char *ssid = "ssid";
const char *password = "pwd";
int ssrPin = D3;    # SSR Pin D3
int tcPin = A0;     # AD595 Pin A0
float tcTemperature;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
const char *PARAM_INPUT_1 = "state";

// ***** TYPE DEFINITIONS *****
typedef enum REFLOW_STATE
{
    REFLOW_STATE_IDLE,
    REFLOW_STATE_PREHEAT,
    REFLOW_STATE_SOAK,
    REFLOW_STATE_REFLOW,
    REFLOW_STATE_COOL,
    REFLOW_STATE_COMPLETE,
    REFLOW_STATE_TOO_HOT,
    REFLOW_STATE_ERROR
} reflowState_t;

typedef enum REFLOW_STATUS
{
    REFLOW_STATUS_OFF,
    REFLOW_STATUS_ON
} reflowStatus_t;

// ***** CONSTANTS *****
#define TEMPERATURE_ROOM 60
#define TEMPERATURE_SOAK_MIN 100
#define TEMPERATURE_SOAK_MAX 150
#define TEMPERATURE_REFLOW_MAX 235
#define TEMPERATURE_COOL_MIN 100
#define SENSOR_SAMPLING_TIME 1000
#define SOAK_TEMPERATURE_STEP 5
#define SOAK_MICRO_PERIOD 9000
#define DEBOUNCE_PERIOD_MIN 50

// ***** PID PARAMETERS *****
// ***** PRE-HEAT STAGE *****
#define PID_KP_PREHEAT 100
#define PID_KI_PREHEAT 0.025
#define PID_KD_PREHEAT 20
// ***** SOAKING STAGE *****
#define PID_KP_SOAK 300
#define PID_KI_SOAK 0.05
#define PID_KD_SOAK 250
// ***** REFLOW STAGE *****
#define PID_KP_REFLOW 300
#define PID_KI_REFLOW 0.05
#define PID_KD_REFLOW 350
#define PID_SAMPLE_TIME 1000

// ***** LCD MESSAGES *****
const char *lcdMessagesReflowStatus[] = {
    "Ready",
    "Pre-heat",
    "Soak",
    "Reflow",
    "Cool",
    "Complete",
    "Wait,hot",
    "Error"};




bool runProcessFlag = false;

// ***** PID CONTROL VARIABLES *****
double setpoint;
double input;
double output;
double kp = PID_KP_PREHEAT;
double ki = PID_KI_PREHEAT;
double kd = PID_KD_PREHEAT;
int windowSize;
unsigned long windowStartTime;
unsigned long nextCheck;
unsigned long nextRead;
unsigned long timerSoak;
unsigned long buzzerPeriod;
// Reflow oven controller state machine state variable
reflowState_t reflowState;
// Reflow oven controller status
reflowStatus_t reflowStatus;
// Seconds timer
int timerSeconds;

// Specify PID control interface
PID reflowOvenPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

bool checkSerial(String cmd)
{
    String input;

    if (Serial.available())
    {
        input = Serial.readStringUntil('\n');

        if (input.equals(cmd))
            return true;
    }

    return false;
}

void serverSetup()
{
    if (!SPIFFS.begin())
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi..");
    }

    // Print ESP32 Local IP Address
    Serial.println(WiFi.localIP());

    // Route for web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send(SPIFFS, "/index.html"); });
    server.on("/temp", HTTP_GET, [](AsyncWebServerRequest *request)
              { request->send_P(200, "text / plain", String(tcTemperature).c_str()); });
    server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request)
              {
                  String inputMessage;
                  String inputParam;
                  // GET input1 value on <ESP_IP>/update?state=<inputMessage>
                  if (request->hasParam(PARAM_INPUT_1))
                  {
                      inputMessage = request->getParam(PARAM_INPUT_1)->value();
                      inputParam = PARAM_INPUT_1;
                      if (inputMessage.toInt() == 1)
                          runProcessFlag = true;
                      else
                          runProcessFlag = false;
                      //digitalWrite(output, inputMessage.toInt());
                  }
                  else
                  {
                      inputMessage = "No message sent";
                      inputParam = "none";
                  }
                  Serial.println(inputMessage);
                  request->send(200, "text/plain", "OK");
              });

    // Send a GET request to <ESP_IP>/state
    server.on("/state", HTTP_GET, [](AsyncWebServerRequest *request)
              {
                  request->send(200, "text/plain", String(runProcessFlag ? "1":"0").c_str());
                  Serial.print("State Requested! -- ");
                  Serial.println(runProcessFlag);
              });
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request)
    {
        String status = "<b>" + String(lcdMessagesReflowStatus[reflowState]) + "</b> | Timer Seconds: <b>" + String(timerSeconds) + "</b> | Set Point: <b>" + String(setpoint) + "</b> | PID output: <b>" + String(output) + "</b>";
        request->send(200, "text/plain", status.c_str());
    });
    
    // start server
    server.begin();
}

void setup()
{
    // SSR pin initialization to ensure reflow oven is off
    digitalWrite(ssrPin, LOW);
    pinMode(ssrPin, OUTPUT);

    Serial.begin(115200);
    Serial.println("Reflow Oven 1.2");
    serverSetup();
    
    // Set window size
    windowSize = 2000;
    // Initialize time keeping variable
    nextCheck = millis();
    // Initialize thermocouple reading variable
    nextRead = millis();
}

void loop()
{
    // Current time
    unsigned long now;
    int adc_val;

    // Time to read thermocouple?
    if (millis() > nextRead)
    {
        // Read thermocouple next sampling period
        nextRead += SENSOR_SAMPLING_TIME;
        // Read current temperature
        adc_val = analogRead(tcPin);
        tcTemperature = (((adc_val * 4.88) - 0.0027) / 10.0);
        input = tcTemperature;
    }

    if (millis() > nextCheck)
    {
        // Check input in the next seconds
        nextCheck += 1000;
        // If reflow process is on going
        if (reflowStatus == REFLOW_STATUS_ON)
        {
            // Increase seconds timer for reflow curve analysis
            timerSeconds++;
            // Send temperature and time stamp to serial
            Serial.print(" Timerseconds: ");
                Serial.print(timerSeconds);
            Serial.print(" ");
            Serial.print(" - Set Point: ");
                Serial.print(setpoint);
            Serial.print(" ");
            Serial.print(" - Current Temp: ");
                Serial.print(input);
            Serial.print(" ");
            Serial.print(" - PID Output Value: ");
                Serial.println(output);
        }

        Serial.print(lcdMessagesReflowStatus[reflowState]);

        // If currently in error state
        if (reflowState == REFLOW_STATE_ERROR)
        {
            // No thermocouple wire connected
            Serial.println("TC Error!");
        }
        else
        {
            // Print current temperature
            Serial.print(" - Current Temp: ");
                Serial.println(input);
        }
    }

    // Reflow oven controller state machine
    switch (reflowState)
    {
    case REFLOW_STATE_IDLE:
        // If oven temperature is still above room temperature
        if (input >= TEMPERATURE_ROOM)
        {
            reflowState = REFLOW_STATE_TOO_HOT;
        }
        else
        {
            // If switch is pressed to start reflow process
            if (checkSerial("start") || runProcessFlag)
            {
                // Send header for CSV file
                Serial.println("Time Setpoint Input Output");
                // Intialize seconds timer for serial debug information
                timerSeconds = 0;
                // Initialize PID control window starting time
                windowStartTime = millis();
                // Ramp up to minimum soaking temperature
                setpoint = TEMPERATURE_SOAK_MIN;
                // Tell the PID to range between 0 and the full window size
                reflowOvenPID.SetOutputLimits(0, windowSize);
                reflowOvenPID.SetSampleTime(PID_SAMPLE_TIME);
                // Turn the PID on
                reflowOvenPID.SetMode(AUTOMATIC);
                // Proceed to preheat stage
                reflowState = REFLOW_STATE_PREHEAT;
            }
        }
        break;

    case REFLOW_STATE_PREHEAT:
        reflowStatus = REFLOW_STATUS_ON;
        // If minimum soak temperature is achieve
        if (input >= TEMPERATURE_SOAK_MIN)
        {
            // Chop soaking period into smaller sub-period
            timerSoak = millis() + SOAK_MICRO_PERIOD;
            // Set less agressive PID parameters for soaking ramp
            reflowOvenPID.SetTunings(PID_KP_SOAK, PID_KI_SOAK, PID_KD_SOAK);
            // Ramp up to first section of soaking temperature
            setpoint = TEMPERATURE_SOAK_MIN + SOAK_TEMPERATURE_STEP;
            // Proceed to soaking state
            reflowState = REFLOW_STATE_SOAK;
        }
        break;

    case REFLOW_STATE_SOAK:
        // If micro soak temperature is achieved
        if (millis() > timerSoak)
        {
            timerSoak = millis() + SOAK_MICRO_PERIOD;
            // Increment micro setpoint
            setpoint += SOAK_TEMPERATURE_STEP;
            if (setpoint > TEMPERATURE_SOAK_MAX)
            {
                // Set agressive PID parameters for reflow ramp
                reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
                // Ramp up to first section of soaking temperature
                setpoint = TEMPERATURE_REFLOW_MAX;
                // Proceed to reflowing state
                reflowState = REFLOW_STATE_REFLOW;
            }
        }
        break;

    case REFLOW_STATE_REFLOW:
        // We need to avoid hovering at peak temperature for too long
        // Crude method that works like a charm and safe for the components
        if (input >= (TEMPERATURE_REFLOW_MAX - 5))
        {
            // Set PID parameters for cooling ramp
            reflowOvenPID.SetTunings(PID_KP_REFLOW, PID_KI_REFLOW, PID_KD_REFLOW);
            // Ramp down to minimum cooling temperature
            setpoint = TEMPERATURE_COOL_MIN;
            // Proceed to cooling state
            reflowState = REFLOW_STATE_COOL;
        }
        break;

    case REFLOW_STATE_COOL:
        // If minimum cool temperature is achieve
        if (input <= TEMPERATURE_COOL_MIN)
        {
            // Retrieve current time for buzzer usage
            buzzerPeriod = millis() + 1000;
            // Turn off reflow process
            reflowStatus = REFLOW_STATUS_OFF;
            // Proceed to reflow Completion state
            reflowState = REFLOW_STATE_COMPLETE;
        }
        break;

    case REFLOW_STATE_COMPLETE:
        if (millis() > buzzerPeriod)
        {
            // Reflow process ended
            reflowState = REFLOW_STATE_IDLE;
            runProcessFlag = false;
        }
        break;

    case REFLOW_STATE_TOO_HOT:
        // If oven temperature drops below room temperature
        if (input < TEMPERATURE_ROOM)
        {
            // Ready to reflow
            reflowState = REFLOW_STATE_IDLE;
        }
        break;
    }

    // If switch 1 is pressed
    if (checkSerial("stop") || !runProcessFlag)
    {
        // If currently reflow process is on going
        if (reflowStatus == REFLOW_STATUS_ON)
        {
            // Button press is for cancelling
            // Turn off reflow process
            reflowStatus = REFLOW_STATUS_OFF;
            // Reinitialize state machine
            reflowState = REFLOW_STATE_IDLE;
        }
    }
    // PID computation and SSR control
    if (reflowStatus == REFLOW_STATUS_ON)
    {
        now = millis();

        reflowOvenPID.Compute();

        if ((now - windowStartTime) > windowSize)
        {
            // Time to shift the Relay Window
            windowStartTime += windowSize;
        }
        if (output > (now - windowStartTime))
            digitalWrite(ssrPin, HIGH);
        else
            digitalWrite(ssrPin, LOW);
    }
    // Reflow oven process is off, ensure oven is off
    else
    {
        digitalWrite(ssrPin, LOW);
    }
}
