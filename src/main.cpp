#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "Adafruit_VL53L0X.h"
#include "html.h"
#include "rom/rtc.h"

const char* getResetReason(RESET_REASON reason) {
    switch (reason) {
        case 1:  return "POWERON";
        case 3:  return "SW_RESET";
        case 7:  return "TG0WDT";
        case 8:  return "TG1WDT";
        case 12: return "SW_CPU_RESET";
        case 15: return "BROWNOUT";
        default: return "UNKNOWN";
    }
}


const char* ssid = "MazeBot";
const char* pass = "mazebot123";


volatile float left_distance = 0; 
volatile float center_distance = 1000;
volatile float right_distance = 0;
volatile float enc_error = 0;
float tof_error = 0;

float last_enc_error = 0;
float last_tof_error = 0;
volatile float pwmerror = 0;
float enc_kp = 8;
float enc_kd = 0;
float enc_ki = 0;
    
float tof_kp = 0.2;
float tof_kd = 1.5;
float tof_ki = 0;

int baseSpeed = 200;
float turning_threshold = 200;

int turnStartEncL = 0;
int turnStartEncR = 0;
int turnStartDiff = 0;
bool turnDirectionRight = false;
int TURN_PULSES = 750;

enum RobotState {
    STOP,
    FOLLOW,
    TURN
};

RobotState robot_state;
WebServer server(80);

int x = 0;

unsigned long lastUpdate = 0;
const unsigned long updateInterval = 2000; // update x every 2 seconds

volatile int encCountL = 0;
volatile int encCountR = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR isrLeft()  { 
    portENTER_CRITICAL_ISR(&mux);
    encCountL++; 
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR isrRight() { 
    portENTER_CRITICAL_ISR(&mux);
    encCountR++; 
    portEXIT_CRITICAL_ISR(&mux);
}
// Encoders (single channel)
const int ENC_L = 34;   
const int ENC_R = 35;   

volatile int motorSpeedL = 0;
volatile int motorSpeedR = 0;

const int L2   = 19;    
const int L1   = 18;    
const int R2   = 17;    
const int R1   = 16;    

const int PWML = 32;    
const int PWMR = 33;    

const int scl = 22;
const int sda = 21;


// XSHUT PINS
const int XSHUT_L = 25;   
const int XSHUT_R = 26;
const int XSHUT_C = 27;

const int freq = 5000;
const int ledcChannelL = 0;
const int ledcChannelR = 1;
const int resolution = 8;


// Define unique I2C addresses for each sensor
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
#define LOX3_ADDRESS 0x29

// Create sensor instances
Adafruit_VL53L0X loxL = Adafruit_VL53L0X();
Adafruit_VL53L0X loxR = Adafruit_VL53L0X();
Adafruit_VL53L0X loxC = Adafruit_VL53L0X();

TaskHandle_t sensorTaskHandle;
TaskHandle_t controlTaskHandle;

const int CONTROL_HZ = 300;

char current_log[64] = "Booting...";

bool sensorsReady = false;
bool NewReading = false;

int min_speed = 120;
int turn_speed = 230;
static bool turnInitialized = false;
int maze_width = 250;
unsigned long TurnStartTime = 0;
int Brake_Duration = 50000;

// Structure to hold Kalman filter states
struct KalmanFilter {
  float q = 20; // Process noise covariance (how much the robot actually drifts/moves)
  float r = 25.00; // Measurement noise covariance (sensor fluctuation variance - adjust this!)
  float x = 0.00; // Estimated value
  float p = 1.00; // Estimation error covariance
  float k = 0.00; // Kalman gain
};

// Initialize filters for left and right sensors
KalmanFilter kfLeft;
KalmanFilter kfRight;
KalmanFilter kfCenter;

// Function to update the Kalman filter with a new raw measurement
float updateKalman(KalmanFilter &kf, float measurement) {
  // 1. Predict Step (Object is stationary, so predicted x remains the same)
  kf.p = kf.p + kf.q;

  // 2. Update Step
  kf.k = kf.p / (kf.p + kf.r);
  kf.x = kf.x + kf.k * (measurement - kf.x);
  kf.p = (1 - kf.k) * kf.p;

  return kf.x;
}

void handleRoot(){
    server.send_P(200, "text/html", html);
}

void handleData() {
    String json = "{\"encL\":" + String(encCountL) + ",\"encR\":" + String(encCountR) + "}";
    server.send(200, "application/json", json);
}

void FormHandler() {
    if (server.hasArg("base_speed"))        baseSpeed          = server.arg("base_speed").toInt();
    if (server.hasArg("min_speed"))         min_speed          = server.arg("min_speed").toInt();
    if (server.hasArg("turn_speed"))        turn_speed         = server.arg("turn_speed").toInt();
    if (server.hasArg("turning_threshold")) turning_threshold  = server.arg("turning_threshold").toFloat();
    if (server.hasArg("braking_threshold")) Brake_Duration  = server.arg("braking_threshold").toInt();
    if (server.hasArg("turn_pulses")) TURN_PULSES  = server.arg("turn_pulses").toInt();
    if (server.hasArg("enc_kp")) {
        enc_kp = server.arg("enc_kp").toFloat();
        enc_kd = server.arg("enc_kd").toFloat();
        enc_ki = server.arg("enc_ki").toFloat();
        tof_kp = server.arg("tof_kp").toFloat();
        tof_kd = server.arg("tof_kd").toFloat();
        tof_ki = server.arg("tof_ki").toFloat();
    }
    server.send(200, "text/plain", "Parameters updated successfully!");
}

void setupAP(){
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, pass);

    server.on("/", HTTP_GET, handleRoot);
    server.on("/updatePID", HTTP_POST, FormHandler);
    server.on("/d", HTTP_GET, []() {
    char buf[256];
    snprintf(buf, sizeof(buf), "%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%s,%d,%d,%d",
        (float)left_distance,
        (float)center_distance,
        (float)right_distance,
        (float)enc_error,
        (float)pwmerror,
        tof_error,
        current_log,
        (int)motorSpeedL,
        (int)motorSpeedR,
        (int)robot_state);
    server.send(200, "text/plain", buf);
});
    server.on("/stop", HTTP_GET, []() {
    robot_state = STOP;
    server.send(200, "text/plain", "Stopped");
    });

    server.on("/follow", HTTP_GET, []() {
    robot_state = FOLLOW;
    server.send(200, "text/plain", "Following");
    });
    
    server.on("/turn", HTTP_GET, []() {
    robot_state = TURN;
    server.send(200, "text/plain", "Turning");
    });

    server.on("/config", HTTP_GET, []() {
    char buf[64];
    snprintf(buf, sizeof(buf), "%d,%d,%d,%.1f,%d,%d",
        baseSpeed, min_speed, turn_speed,
        turning_threshold, Brake_Duration, TURN_PULSES);
    server.send(200, "text/plain", buf);
});

    server.begin();
    Serial.println("HTTP server started");
}

void handleNotFound() {
    server.send(404, "text/plain", "Not found");
}

void setID(){
  pinMode(XSHUT_L, OUTPUT);
  pinMode(XSHUT_R, OUTPUT);
  pinMode(XSHUT_C, OUTPUT);
  digitalWrite(XSHUT_L, LOW);
  digitalWrite(XSHUT_R, LOW);
  digitalWrite(XSHUT_C, LOW);
  delay(10);

  pinMode(XSHUT_L, INPUT);
  delay(10);

  if (!loxL.begin(LOX1_ADDRESS, true, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED)) {
    Serial.println("Failed to boot first VL53L0X");
    while (1);
  }else{
    Serial.println("First VL53L0X initialized successfully");
  }
  delay(10);

  pinMode(XSHUT_R, INPUT);
  delay(10);

  if (!loxR.begin(LOX2_ADDRESS, true, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED)) {
    Serial.println("Failed to boot second VL53L0X");
    while (1);
  } else {
    Serial.println("Second VL53L0X initialized successfully");
  }

  pinMode(XSHUT_C, INPUT);  
  delay(10);

  if (!loxC.begin(LOX3_ADDRESS, true, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED)) {
    Serial.println("Failed to boot second VL53L0X");
    while (1);
  } else {
    Serial.println("Third VL53L0X initialized successfully");
  }
  loxL.startRangeContinuous();
  loxR.startRangeContinuous();
  loxC.startRangeContinuous();
  sensorsReady = true;

}

void EncoderPID(int startErr){
    enc_error = (encCountL - encCountR) - startErr;
    float delta = enc_error - last_enc_error;
    
    float correction = (enc_kp * enc_error) + (enc_kd * delta);
    last_enc_error = enc_error;
    pwmerror = correction;

    motorSpeedL  = constrain((int)(turn_speed - correction), 0, 255);
    motorSpeedR = constrain((int)(turn_speed + correction), 0, 255);
    ledcWrite(ledcChannelL, motorSpeedL);
    ledcWrite(ledcChannelR, motorSpeedR);
}

void tofPID() {
    static float delta = 0;
    static int lastMode = -1;
    int currentmode;
    if(left_distance < 230 && right_distance < 230){
        currentmode = 0;
        tof_error = (left_distance - right_distance);
        
    } else if(left_distance >230){
        currentmode = 1;
        strcpy(current_log, "Switched to right ToF correction!");
        tof_error = ((maze_width/2)-(22.5) - right_distance);
        
    } else{
        currentmode = 2;
        strcpy(current_log, "Switched to left ToF correction");
        tof_error = left_distance - ((maze_width/2)-(22.5) );
        
    }
    
    if(currentmode != lastMode) {
        last_tof_error = tof_error;
        delta = 0;
        lastMode = currentmode;
        NewReading = false;  
    } else if(NewReading) {
            delta = tof_error - last_tof_error;
            last_tof_error = tof_error;
            NewReading = false;  
        }
    pwmerror = (tof_kp * tof_error) + (tof_kd * delta);

    int leftMotorSpeed = baseSpeed - pwmerror;
    int rightMotorSpeed = baseSpeed + pwmerror;

    motorSpeedL = constrain((int)leftMotorSpeed, 0, 255);
    motorSpeedR = constrain((int)rightMotorSpeed, 0, 255);
    ledcWrite(ledcChannelL, motorSpeedL);
    ledcWrite(ledcChannelR, motorSpeedR);
}

void readTOF(){
  if(loxL.isRangeComplete()) {
        uint16_t rawL = loxL.readRangeResult();
        NewReading = true;
        if(rawL < 2000)
            left_distance = constrain(updateKalman(kfLeft, rawL), 0, 1000);
        else
            left_distance = 1000;
    }

    if(loxR.isRangeComplete()) {
        uint16_t rawR = loxR.readRangeResult();
        NewReading = true;
        if(rawR < 2000)
            right_distance = constrain(updateKalman(kfRight, rawR), 0, 1000);
        else
            right_distance = 1000;
    }

    if(loxC.isRangeComplete()) {
        uint16_t rawC = loxC.readRangeResult();
        NewReading = true;  
        if(rawC < 2000)
            center_distance = constrain(updateKalman(kfCenter, rawC), 0, 1000);
        else
            center_distance = 1000;
    }

}

void loop() {}

void Turning_Logic(){
        if(!turnInitialized) {

            turnStartEncL = encCountL;
            turnStartEncR = encCountR;
            TurnStartTime = micros();
            turnStartDiff = encCountL - encCountR;  
            last_enc_error = 0;
            turnDirectionRight = (right_distance > left_distance); 
            turnInitialized  = true;
        }
        
        if(turnDirectionRight && micros() < TurnStartTime + Brake_Duration){
            digitalWrite(R1, HIGH); digitalWrite(R2, LOW);   
            ledcWrite(ledcChannelR, turn_speed);
            ledcWrite(ledcChannelL, 0);
            return;
        } else if(!turnDirectionRight && micros() < TurnStartTime + Brake_Duration ){
            digitalWrite(L1, LOW);  digitalWrite(L2, HIGH);  
            ledcWrite(ledcChannelL, turn_speed);
            ledcWrite(ledcChannelR, 0);
            return;
        }


        int travelledL = encCountL - turnStartEncL;
        int travelledR = encCountR - turnStartEncR;
        
        if(turnDirectionRight){
            digitalWrite(L1, HIGH); digitalWrite(L2, LOW);  
            ledcWrite(ledcChannelL, turn_speed);
            ledcWrite(ledcChannelR, 0);
            if(travelledL >= TURN_PULSES) {
                turnInitialized = false;
                last_tof_error = 0;
                tof_error = 0;
                snprintf(current_log, sizeof(current_log), "Turn Complete! Time: %lu", micros() - TurnStartTime);
                robot_state = FOLLOW;
            }   
        }else {
            digitalWrite(R1, LOW);  digitalWrite(R2, HIGH);  
            ledcWrite(ledcChannelL, 0);
            ledcWrite(ledcChannelR, turn_speed);
            if(travelledR >= TURN_PULSES) {
                snprintf(current_log, sizeof(current_log), "Turn Complete! Time: %lu", micros() - TurnStartTime);
                last_tof_error = 0;
                turnInitialized = false; 
                tof_error = 0;
                robot_state = FOLLOW;
            }  
        }

        
    
}

void taskSensorCore(void* pvParameters){
    setID();
    setupAP();

    for(;;) {
        readTOF();
        server.handleClient();
        vTaskDelay(pdMS_TO_TICKS(5));
      }
}

void taskControlCore(void* pvParameters){
    while (!sensorsReady) {
    vTaskDelay(pdMS_TO_TICKS(10)); 
}
    const TickType_t xFrequency = pdMS_TO_TICKS(1000 / CONTROL_HZ);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    robot_state = STOP;
    for(;;) {
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      if(robot_state==FOLLOW && center_distance<((maze_width/2) + 42.5) && (left_distance > 200 || right_distance > 200)){

        robot_state = TURN;
        strcpy(current_log, "Switching to turn");
      }
      switch(robot_state){
        case FOLLOW:
            digitalWrite(L1, HIGH);
            digitalWrite(L2, LOW);
            digitalWrite(R1, LOW); 
            digitalWrite(R2, HIGH);
            
            tofPID();
            break;
        case TURN:
            Turning_Logic();
            break;
        case STOP:
            ledcWrite(ledcChannelL, 0);
            ledcWrite(ledcChannelR, 0);
            break;
  }    
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin(sda, scl);
    snprintf(current_log, sizeof(current_log),
         "RST C0:%s C1:%s",
         getResetReason(rtc_get_reset_reason(0)),
         getResetReason(rtc_get_reset_reason(1)));

    ledcSetup(ledcChannelL, freq, resolution);
    ledcSetup(ledcChannelR, freq, resolution);
    
    ledcAttachPin(PWML, ledcChannelL);
    ledcAttachPin(PWMR, ledcChannelR);

    pinMode(ENC_L, INPUT_PULLUP);
    pinMode(ENC_R, INPUT_PULLUP);

    pinMode(L1, OUTPUT); pinMode(L2, OUTPUT);
    pinMode(R1, OUTPUT); pinMode(R2, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(ENC_L), isrLeft,  RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R), isrRight, RISING);
    
    xTaskCreatePinnedToCore(
    taskSensorCore,    // function
    "SensorCore",      // name
    8192,              // stack size (bytes) — WiFi needs generous stack
    NULL,              // parameters
    20,                 // priority
    &sensorTaskHandle, // handle
    0                  // core 0 — WiFi lives here
    );

xTaskCreatePinnedToCore(
    taskControlCore,    // function
    "ControlCore",      // name
    4096,               // stack size
    NULL,               // parameters
    1,                  // priority
    &controlTaskHandle, // handle
    1                   // core 1 — control lives here
);
}

