#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "Adafruit_VL53L0X.h"
#include "html.h"
#include "rom/rtc.h"

String getResetReason(RESET_REASON reason) {
    switch (reason) {
        case 1:  return "POWERON";
        case 3:  return "SW_RESET";
        case 7:  return "TG0WDT";      // task watchdog core 0
        case 8:  return "TG1WDT";      // task watchdog core 1
        case 12: return "SW_CPU_RESET";
        case 15: return "BROWNOUT";    // power issue
        default: return "UNKNOWN(" + String(reason) + ")";
    }
}


const char* ssid = "MazeBot";
const char* pass = "mazebot123";


volatile float left_distance = 0; 
volatile float center_distance = 100;
volatile float right_distance = 0;
volatile float enc_error = 0;
float tof_error = 0;

float last_enc_error = 0;
float last_tof_error = 0;
volatile float pwmerror = 0;
float enc_kp = 8;
float enc_kd = 0;
float enc_ki = 0;
    
float tof_kp = 1.1;
float tof_kd = 45;
float tof_ki = 0;

int baseSpeed = 230;
float braking_threshold = 350;
float turning_threshold = 200;

int turnStartEncL = 0;
int turnStartEncR = 0;
int turnStartDiff = 0;
bool turnDirectionRight = false;
int TURN_PULSES = 380;

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

const int L1   = 19;    
const int L2   = 18;    
const int R1   = 16;    
const int R2   = 17;    

const int PWML = 32;    
const int PWMR = 33;    

const int scl = 22;
const int sda = 21;

// XSHUT PINS
const int XSHUT_L = 25;   
const int XSHUT_R = 26;
const int XSHUT_C = 4;

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

String current_log = "Booting...";

bool sensorsReady = false;

int decel = 0;
unsigned int last_decel_time =0;
int min_speed = 120;
int turn_speed = 180;

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
  if (server.hasArg("base_speed")) {
    baseSpeed = server.arg("base_speed").toInt();
  }

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
        String data = String(left_distance, 1) + "," + 
                    String(center_distance, 1) + "," + 
                    String(right_distance, 1) + "," + 
                    String(enc_error, 1) + "," + 
                    String(pwmerror, 1) + "," + 
                    String(tof_error, 1) + "," +
                    current_log + " L:" + String(encCountL) + " R:" + String(encCountR);
        server.send(200, "text/plain", data);
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

    server.on("/setPulses", HTTP_GET, []() {
    if(server.hasArg("v")) {
        TURN_PULSES = server.arg("v").toInt();
        server.send(200, "text/plain", "Pulses set to " + String(TURN_PULSES));
    } else {
        server.send(400, "text/plain", "Missing ?v=");
    }
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
    enc_error = encCountL - encCountR - startErr;
    float delta = enc_error - last_enc_error;
    
    pwmerror = (enc_kp * enc_error) + (enc_kd * delta);

    last_enc_error = enc_error;

    int leftMotorSpeed = baseSpeed - pwmerror;
    int rightMotorSpeed = baseSpeed + pwmerror;

    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    ledcWrite(ledcChannelL, leftMotorSpeed);
    ledcWrite(ledcChannelR, rightMotorSpeed);
}

void tofPID() {
    float target = 15.0; 
    tof_error = (left_distance - right_distance); 
    float delta = tof_error - last_tof_error;


    pwmerror = (tof_kp * tof_error) + (tof_kd * delta);
    last_tof_error = tof_error;

    int leftMotorSpeed = baseSpeed - pwmerror;
    int rightMotorSpeed = baseSpeed + pwmerror;

    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    ledcWrite(ledcChannelL, leftMotorSpeed);
    ledcWrite(ledcChannelR, rightMotorSpeed);
}

void readTOF(){
  if(loxL.isRangeComplete() && loxR.isRangeComplete()) {
    left_distance = constrain(updateKalman(kfLeft, loxL.readRangeResult()), 0, 1000);
    right_distance = constrain(updateKalman(kfRight, loxR.readRangeResult()), 0, 1000);
    center_distance = constrain(updateKalman(kfCenter, loxC.readRangeResult()), 0, 1000);
  } 
}

void loop() {}

void Turning_Logic(){
    if (center_distance>turning_threshold) {
        if (baseSpeed>min_speed) {
            baseSpeed -= decel*(micros() - last_decel_time)* 1e-6;
        }
        digitalWrite(L1, HIGH);
        digitalWrite(L2, LOW);
        digitalWrite(R1, LOW); 
        digitalWrite(R2, HIGH);
        tofPID();
    } else {
        static bool turnInitialized = false;
        if(!turnInitialized) {
            turnStartEncL = encCountL;
            turnStartEncR = encCountR;
            turnStartDiff = encCountL - encCountR;  
            last_enc_error = turnStartDiff;
            turnDirectionRight = (right_distance > left_distance); 
            turnInitialized  = true;
        }

        int travelledL = encCountL - turnStartEncL;
        int travelledR = encCountR - turnStartEncR;
        
        if(turnDirectionRight){
            digitalWrite(L1, HIGH); digitalWrite(L2, LOW);  
            digitalWrite(R1, HIGH); digitalWrite(R2, LOW);   
            baseSpeed=turn_speed;
            EncoderPID(turnStartDiff);
        }else {
            digitalWrite(L1, LOW);  digitalWrite(L2, HIGH);  
            digitalWrite(R1, LOW);  digitalWrite(R2, HIGH);  
            baseSpeed=turn_speed;
            EncoderPID(turnStartDiff);
        }

        if(travelledL >= TURN_PULSES && travelledR >= TURN_PULSES) {
            turnInitialized = false; 
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
      if(robot_state!=TURN && center_distance<braking_threshold){
        robot_state = TURN;
        last_decel_time = micros();
      }
      switch(robot_state){
        case FOLLOW:
            baseSpeed=230;
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
    delay(2000);
    Wire.begin(sda, scl);
    current_log = "RST C0:" + getResetReason(rtc_get_reset_reason(0)) +
                  " C1:" + getResetReason(rtc_get_reset_reason(1));

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
    24,                 // priority
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

