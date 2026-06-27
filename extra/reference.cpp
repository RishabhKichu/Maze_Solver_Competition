/*
 * ============================================================
 *  HIGH-SPEED WALL FOLLOWING ROBOT — PHASE 1
 *  Dual-core ESP32 | FreeRTOS | VL53L0X x3 via TCA9548A
 *  Core 0: Sensor IO + MQTT telemetry
 *  Core 1: Steering PID + Motor control + Encoder ISR
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ============================================================
//  TUNING VARIABLES — adjust these without touching logic
// ============================================================

// --- WiFi & MQTT ---
const char* WIFI_SSID         = "your_wifi_ssid";
const char* WIFI_PASS         = "your_wifi_password";
const char* MQTT_BROKER       = "192.168.x.x";   // your laptop IP
const int   MQTT_PORT         = 1883;

// --- Maze geometry ---
const float WALL_SPACING_CM   = 16.0f;   // wall-to-wall width
const float WALL_CENTER_CM    = WALL_SPACING_CM / 2.0f;  // ideal side dist

// --- End of run detection ---
const float END_DIST_THRESHOLD = 60.0f;  // cm — abnormally large reading
const int   END_CONFIRM_MS     = 300;    // ms both sides must stay large

// --- Steering PID (runs on Core 1 at high rate) ---
const float STEER_KP          = 8.0f;
const float STEER_KD          = 2.5f;
// No KI for wall following — integrator windup causes oscillation

// --- Speed ---
const int   BASE_PWM          = 160;     // 0–255
const int   MIN_PWM           = 80;
const int   MAX_PWM           = 230;

// --- Front wall braking ---
const float FRONT_STOP_CM     = 12.0f;   // stop/steer if front wall closer

// --- Control loop rate ---
const int   CONTROL_HZ        = 400;     // Core 1 loop rate (Hz)
const int   SENSOR_HZ         = 40;      // Core 0 ToF poll rate (Hz)
const int   TELEMETRY_HZ      = 20;      // Core 0 MQTT publish rate (Hz)

// ============================================================
//  PIN DEFINITIONS
// ============================================================

// Motor driver (L298N)
const int L1   = 25;   // left forward
const int L2   = 26;   // left backward
const int R1   = 27;   // right forward
const int R2   = 14;   // right backward
const int PWML = 32;   // left PWM
const int PWMR = 33;   // right PWM

// Encoders (single channel)
const int ENC_L = 34;
const int ENC_R = 35;

// ============================================================
//  TCA9548A I2C MULTIPLEXER
// ============================================================
const int TCA_ADDR = 0x70;
const int CH_LEFT = 0;
const int CH_FRONT = 1;
const int CH_RIGHT = 2;

void tcaSelect(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

// ============================================================
//  SHARED STATE — Core 0 writes, Core 1 reads
// ============================================================
volatile struct SensorState {
    float leftDist;
    float frontDist;
    float rightDist;
    bool  valid;        // true once first readings are in
} sensors = {20.0f, 50.0f, 20.0f, false};

// Core 1 writes these for telemetry, Core 0 reads them
volatile struct ControlState {
    float steerError;
    int   pwmLeft;
    int   pwmRight;
    int   encLeft;
    int   encRight;
    bool  running;
} ctrl = {0, 0, 0, 0, 0, true};

// ============================================================
//  ROBOT STATE MACHINE
// ============================================================
enum RobotState { RUNNING, FINISHED };
volatile RobotState robotState = RUNNING;

// ============================================================
//  ENCODER ISR
// ============================================================
volatile int encCountL = 0;
volatile int encCountR = 0;

void IRAM_ATTR isrLeft()  { encCountL++; }
void IRAM_ATTR isrRight() { encCountR++; }

// ============================================================
//  MOTOR HELPERS
// ============================================================
void motorsStop() {
    analogWrite(PWML, 0);
    analogWrite(PWMR, 0);
    digitalWrite(L1, LOW); digitalWrite(L2, LOW);
    digitalWrite(R1, LOW); digitalWrite(R2, LOW);
}

void motorsForward(int pwmL, int pwmR) {
    digitalWrite(L1, HIGH); digitalWrite(L2, LOW);
    digitalWrite(R1, HIGH); digitalWrite(R2, LOW);
    analogWrite(PWML, constrain(pwmL, 0, 255));
    analogWrite(PWMR, constrain(pwmR, 0, 255));
}

// ============================================================
//  VL53L0X SENSORS
// ============================================================
VL53L0X tofLeft, tofFront, tofRight;

bool initToF(VL53L0X &sensor, uint8_t channel) {
    tcaSelect(channel);
    delay(10);
    if (!sensor.init()) return false;
    sensor.setTimeout(20);
    sensor.startContinuous(0);
    return true;
}

float readToF(VL53L0X &sensor, uint8_t channel) {
    tcaSelect(channel);
    uint16_t mm = sensor.readRangeContinuousMillimeters();
    if (sensor.timeoutOccurred() || mm > 1200) return 100.0f;
    return mm / 10.0f;  // convert to cm
}

// ============================================================
//  WIFI & MQTT
// ============================================================
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);

void connectWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    int tries = 0;
    while (WiFi.status() != WL_CONNECTED && tries < 20) {
        delay(500);
        tries++;
    }
}

void connectMQTT() {
    if (mqtt.connected()) return;
    int tries = 0;
    while (!mqtt.connected() && tries < 5) {
        mqtt.connect("WallBot");
        delay(500);
        tries++;
    }
}

void publishTelemetry() {
    if (!mqtt.connected()) return;

    char buf[128];

    // ToF distances
    snprintf(buf, sizeof(buf), "%.1f,%.1f,%.1f",
        (float)sensors.leftDist,
        (float)sensors.frontDist,
        (float)sensors.rightDist);
    mqtt.publish("robot/tof", buf);

    // PID + motors
    snprintf(buf, sizeof(buf), "%.2f,%d,%d",
        (float)ctrl.steerError,
        (int)ctrl.pwmLeft,
        (int)ctrl.pwmRight);
    mqtt.publish("robot/control", buf);

    // Encoders
    snprintf(buf, sizeof(buf), "%d,%d",
        (int)ctrl.encLeft,
        (int)ctrl.encRight);
    mqtt.publish("robot/encoders", buf);

    // State
    mqtt.publish("robot/state",
        robotState == FINISHED ? "FINISHED" : "RUNNING");
}

// ============================================================
//  CORE 0 TASK — Sensor IO + Telemetry
// ============================================================
void taskSensorCore(void* pvParameters) {
    // Init ToF sensors
    bool ok = true;
    ok &= initToF(tofLeft,  CH_LEFT);
    ok &= initToF(tofFront, CH_FRONT);
    ok &= initToF(tofRight, CH_RIGHT);

    // Connect WiFi + MQTT
    connectWiFi();
    mqtt.setServer(MQTT_BROKER, MQTT_PORT);
    connectMQTT();

    const TickType_t sensorPeriod    = pdMS_TO_TICKS(1000 / SENSOR_HZ);
    const TickType_t telemetryPeriod = pdMS_TO_TICKS(1000 / TELEMETRY_HZ);
    TickType_t lastSensor    = xTaskGetTickCount();
    TickType_t lastTelemetry = xTaskGetTickCount();

    // End-of-run detection
    unsigned long endDetectStart = 0;
    bool endDetecting = false;

    for (;;) {
        TickType_t now = xTaskGetTickCount();

        // --- Sensor reads at SENSOR_HZ ---
        if ((now - lastSensor) >= sensorPeriod) {
            lastSensor = now;

            float l = readToF(tofLeft,  CH_LEFT);
            float f = readToF(tofFront, CH_FRONT);
            float r = readToF(tofRight, CH_RIGHT);

            sensors.leftDist  = l;
            sensors.frontDist = f;
            sensors.rightDist = r;
            sensors.valid     = true;

            // End of run detection — both sides go large
            if (l > END_DIST_THRESHOLD && r > END_DIST_THRESHOLD) {
                if (!endDetecting) {
                    endDetectStart   = millis();
                    endDetecting     = true;
                } else if (millis() - endDetectStart > END_CONFIRM_MS) {
                    robotState = FINISHED;
                }
            } else {
                endDetecting = false;
            }

            // Update ctrl snapshot for telemetry
            ctrl.encLeft  = encCountL;
            ctrl.encRight = encCountR;
        }

        // --- Telemetry at TELEMETRY_HZ ---
        if ((now - lastTelemetry) >= telemetryPeriod) {
            lastTelemetry = now;
            mqtt.loop();
            if (!mqtt.connected()) connectMQTT();
            publishTelemetry();
        }

        taskYIELD();
    }
}

// ============================================================
//  CORE 1 TASK — Steering PID + Motor control
// ============================================================
void taskControlCore(void* pvParameters) {
    const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_HZ);
    TickType_t lastWake     = xTaskGetTickCount();

    float prevError = 0.0f;

    for (;;) {
        vTaskDelayUntil(&lastWake, period);

        if (robotState == FINISHED) {
            motorsStop();
            ctrl.running = false;
            continue;
        }

        if (!sensors.valid) continue;

        float l = sensors.leftDist;
        float f = sensors.frontDist;
        float r = sensors.rightDist;

        // --- Front wall check ---
        if (f < FRONT_STOP_CM) {
            motorsStop();
            continue;
        }

        // --- Steering error ---
        // Both walls visible: center between them
        // One wall visible: maintain setpoint from that wall
        float error = 0.0f;

        bool leftVisible  = (l < END_DIST_THRESHOLD);
        bool rightVisible = (r < END_DIST_THRESHOLD);

        if (leftVisible && rightVisible) {
            error = l - r;                          // center between walls
        } else if (leftVisible) {
            error = l - WALL_CENTER_CM;             // hold distance from left
        } else if (rightVisible) {
            error = WALL_CENTER_CM - r;             // hold distance from right
        }
        // if neither wall visible — go straight (error stays 0)

        // --- PD controller ---
        float derivative = error - prevError;
        float steerOutput = (STEER_KP * error) + (STEER_KD * derivative);
        prevError = error;

        // --- Apply to motors ---
        int pwmL = constrain((int)(BASE_PWM - steerOutput), MIN_PWM, MAX_PWM);
        int pwmR = constrain((int)(BASE_PWM + steerOutput), MIN_PWM, MAX_PWM);

        motorsForward(pwmL, pwmR);

        // --- Update shared state for telemetry ---
        ctrl.steerError = error;
        ctrl.pwmLeft    = pwmL;
        ctrl.pwmRight   = pwmR;
    }
}

// ============================================================
//  SETUP
// ============================================================
void setup() {
    // Motor pins
    pinMode(L1, OUTPUT); pinMode(L2, OUTPUT);
    pinMode(R1, OUTPUT); pinMode(R2, OUTPUT);
    pinMode(PWML, OUTPUT); pinMode(PWMR, OUTPUT);
    motorsStop();

    // Encoder pins
    pinMode(ENC_L, INPUT_PULLUP);
    pinMode(ENC_R, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_L), isrLeft,  RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_R), isrRight, RISING);

    // I2C
    Wire.begin(21, 22);
    Wire.setClock(400000);  // 400kHz fast mode

    // Launch Core 0 task — sensor + telemetry
    xTaskCreatePinnedToCore(
        taskSensorCore,
        "SensorCore",
        8192,           // stack size
        NULL,
        1,              // priority
        NULL,
        0               // Core 0
    );

    // Launch Core 1 task — control
    xTaskCreatePinnedToCore(
        taskControlCore,
        "ControlCore",
        4096,
        NULL,
        2,              // higher priority than sensor task
        NULL,
        1               // Core 1
    );
}

void loop() {
    // Empty — FreeRTOS tasks handle everything
    vTaskDelete(NULL);
}