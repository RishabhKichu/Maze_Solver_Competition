#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "Adafruit_VL53L0X.h"
#include "html.h"

const char* ssid = "MazeBot";
const char* pass = "mazebot123";

float left_distance = 0; 
float center_distance = 0;
float right_distance = 0;
float enc_error = 0;
float tof_error = 0;

float last_enc_error = 0;
float pwmerror = 0;
float enc_kp = 8;
float enc_kd = 0;
float enc_ki = 0;
    
float tof_kp = 0;
float tof_kd = 0;
float tof_ki = 0;

int baseSpeed = 100;

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

const int freq = 5000;
const int ledcChannelL = 0;
const int ledcChannelR = 1;
const int resolution = 8;


// Define unique I2C addresses for each sensor
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31

// Create sensor instances
Adafruit_VL53L0X loxL = Adafruit_VL53L0X();
Adafruit_VL53L0X loxR = Adafruit_VL53L0X();

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

    Serial.print("Base Speed: "); Serial.println(baseSpeed);
    Serial.print("Enc Kp: "); Serial.println(enc_kp);
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
                     String(tof_error, 1);
        server.send(200, "text/plain", data);
    });
    server.begin();
    Serial.println("HTTP server started");
}

void handleNotFound() {
    server.send(404, "text/plain", "Not found");
}

void setID(){
  // 1. Reset both sensors by pulling XSHUT low
  pinMode(XSHUT_L, OUTPUT);
  pinMode(XSHUT_R, OUTPUT);
  digitalWrite(XSHUT_L, LOW);
  digitalWrite(XSHUT_R, LOW);
  delay(100);

  // 2. Un-reset Sensor 1 (keep Sensor 2 in reset)
  pinMode(XSHUT_L, INPUT);
  delay(100);

  // Initialize Sensor 1 and change its address
  if (!loxL.begin(LOX1_ADDRESS, true, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED)) {
    Serial.println("Failed to boot first VL53L0X");
    while (1);
  }else{
    Serial.println("First VL53L0X initialized successfully");
  }
  delay(100);

  // 3. Un-reset Sensor 2
  pinMode(XSHUT_R, INPUT);
  delay(100);

  // Initialize Sensor 2 and change its address
  if (!loxR.begin(LOX2_ADDRESS, true, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED)) {
    Serial.println("Failed to boot second VL53L0X");
    while (1);
  } else {
    Serial.println("Second VL53L0X initialized successfully");
  }
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Wire.begin(sda, scl);
    setID();
    setupAP();

    ledcSetup(ledcChannelL, freq, resolution);
    ledcSetup(ledcChannelR, freq, resolution);
    
    ledcAttachPin(PWML, ledcChannelL);
    ledcAttachPin(PWMR, ledcChannelR);

    pinMode(ENC_L, INPUT_PULLUP);
    pinMode(ENC_R, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L), isrLeft,  RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R), isrRight, RISING);

    pinMode(L1, OUTPUT); pinMode(L2, OUTPUT);
    pinMode(R1, OUTPUT); pinMode(R2, OUTPUT);

    loxL.startRangeContinuous();
    loxR.startRangeContinuous();
}

void EncoderPID(){
    enc_error = encCountL - encCountR;
    float delta = enc_error - last_enc_error;
    
    pwmerror = (enc_kp * enc_error) + (enc_kd * delta);

    int leftMotorSpeed = baseSpeed - pwmerror;
    int rightMotorSpeed = baseSpeed + pwmerror;

    leftMotorSpeed = constrain(leftMotorSpeed, 0, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, 0, 255);

    ledcWrite(ledcChannelL, leftMotorSpeed);
    ledcWrite(ledcChannelR, rightMotorSpeed);
    last_enc_error = enc_error;
}

void readTOF(){
  if(loxL.isRangeComplete() && loxR.isRangeComplete()) {
    left_distance = loxL.readRange();
    right_distance = loxR.readRange();
  } else {
    Serial.println("Error: Range not complete for one or both sensors.");
  }
}


void loop() {
    server.handleClient();

    // digitalWrite(L1, HIGH);
    // digitalWrite(L2, LOW);
    // digitalWrite(R1, HIGH);
    // digitalWrite(R2, LOW);
    // EncoderPID();
    readTOF();

}

