#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h> // 1. Include the DNS Server library
#include <LittleFS.h>

const char* ssid = "MazeBot";
const char* pass = "mazebot123";

float left_sensor = 12.4; 
float center_sensor = 45.1;
float right_sensor = 15.0;
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
DNSServer dnsServer; // 2. Create the DNS Server object
const byte DNS_PORT = 53;

int x = 0;

unsigned long lastUpdate = 0;
const unsigned long updateInterval = 2000; // non-blocking interval for distance logging

volatile int encCountL = 0;
volatile int encCountR = 0;

void IRAM_ATTR isrLeft()  { encCountL++; }
void IRAM_ATTR isrRight() { encCountR++; }

// Encoders (single channel)
const int ENC_L = 34;
const int ENC_R = 35;

const int L1   = 25;   // left forward
const int L2   = 26;   // left backward
const int R1   = 27;   // right forward
const int R2   = 13;   // right backward
const int PWML = 32;   // left PWM
const int PWMR = 33;   // right PWM

const int stby = 14;   // standby pin

const int freq = 5000;
const int ledcChannelL = 0;
const int ledcChannelR = 1;
const int resolution = 8;

const char html[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>MazeBot </title>
  <style>
    body { font-family: sans-serif; text-align: center; margin: 0; padding: 15px; background: #f0f0f0; color: #333; }
    .wrap { display: flex; flex-wrap: wrap; justify-content: center; gap: 15px; max-width: 800px; margin: 0 auto; }
    .card { flex: 1; min-width: 280px; background: #fff; padding: 15px; border-radius: 6px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
    .sect { background: #f9f9f9; margin: 10px 0; padding: 10px; border-radius: 4px; border-left: 4px solid #4CAF50; text-align: left; }
    h2, h4 { margin: 5px 0; }
    .row { margin: 6px 0; display: flex; justify-content: space-between; align-items: center; }
    input[type=number] { padding: 4px; width: 70px; border: 1px solid #ccc; border-radius: 4px; }
    .btn { padding: 8px; background: #4CAF50; color: #fff; border: none; border-radius: 4px; cursor: pointer; width: 100%; font-weight: bold; }
    .btn:hover { background: #45a049; }
    #msg { margin-top: 5px; font-size: 0.9em; color: #4CAF50; font-weight: bold; }
  </style>
</head>
<body>

  <h2>MazeBot Dashboard</h2>
  <div class="wrap">
    
    <div class="card">
      <h2>Sensors</h2>
      <div style="text-align: left; padding: 5px; font-size: 1.1em;">
        <p>Left ToF: <strong id="s_l">0</strong> cm</p>
        <p>Center ToF: <strong id="s_c">0</strong> cm</p>
        <p>Right ToF: <strong id="s_r">0</strong> cm</p>
        <p>Encoder Error: <strong id="s_enc_e">0</strong></p>
        <p>PWM Error: <strong id="s_e1">0</strong></p>
        <p>Tof Error: <strong id="s_e2">0</strong></p>
      </div>
    </div>
    
    <div class="card">
      <h2>Control</h2>
      <form id="f">
        
        <div class="sect">
          <h4>Configuration</h4>
          <div class="row"><label>Base Speed:</label><input type="number" name="base_speed" min="0" max="255" step="1" value="100"></div>
        </div>

        <div class="sect">
          <h4>Encoder PID</h4>
          <div class="row"><label>Kp:</label><input type="number" name="enc_kp" step="0.01" value="8.00"></div>
          <div class="row"><label>Kd:</label><input type="number" name="enc_kd" step="0.01" value="0.00"></div>
          <div class="row"><label>Ki:</label><input type="number" name="enc_ki" step="0.01" value="0.00"></div>
        </div>
        
        <div class="sect">
          <h4>ToF PID</h4>
          <div class="row"><label>Kp:</label><input type="number" name="tof_kp" step="0.01" value="0.00"></div>
          <div class="row"><label>Kd:</label><input type="number" name="tof_kd" step="0.01" value="0.00"></div>
          <div class="row"><label>Ki:</label><input type="number" name="tof_ki" step="0.01" value="0.00"></div>
        </div>
        
        <input type="submit" class="btn" value="Update Parameters">
        <div id="msg"></div>
      </form>
    </div>

  </div>

  <script>
    setInterval(() => {
      fetch('/d')
        .then(r => r.text())
        .then(t => {
          const d = t.split(',');
          if(d.length >= 6) {
            document.getElementById('s_l').innerText = d[0];
            document.getElementById('s_c').innerText = d[1];
            document.getElementById('s_r').innerText = d[2];
            document.getElementById('s_enc_e').innerText = d[3];
            document.getElementById('s_e1').innerText = d[4];
            document.getElementById('s_e2').innerText = d[5];
          }
        }).catch(()=>{});
    }, 100);

    document.getElementById('f').addEventListener('submit', function(e) {
      e.preventDefault();
      fetch('/updatePID', { method: 'POST', body: new URLSearchParams(new FormData(this)) })
        .then(r => r.text())
        .then(t => {
          const m = document.getElementById('msg');
          m.innerText = t;
          setTimeout(() => m.innerText = "", 2000);
        });
    });
  </script>

</body>
</html>
)=====";

void handleRoot(){
    server.send_P(200, "text/html", html);
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

    // 3. Start the DNS Server redirecting all requests ("*") to the AP IP address
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    Serial.print("DNS Server started. AP IP: "); Serial.println(WiFi.softAPIP());

    // CHANGE: Created endpoint handler route to pipe the embedded Chart.js binary data over to the client browser
    server.on("/chart.js", HTTP_GET, []() {
        // Appends custom HTTP Response content encoding headers to flag the dataset as a Gzipped JavaScript file asset
        server.sendHeader("Content-Encoding", "gzip");
        server.send_P(200, "application/javascript", (const char*)chart_js_gz, chart_js_gz_len);
    });

    server.on("/", HTTP_GET, handleRoot);
    server.on("/updatePID", HTTP_POST, FormHandler);
    server.on("/d", HTTP_GET, []() {
        String data = String(left_sensor, 1) + "," + 
                     String(center_sensor, 1) + "," + 
                     String(right_sensor, 1) + "," + 
                     String(enc_error, 1) + "," + 
                     String(pwmerror, 1) + "," + 
                     String(tof_error, 1);
        server.send(200, "text/plain", data);
    });

    
    // Catch-all handler for redirecting unhandled requests (like random URLs) to root
    server.onNotFound([]() {
        server.send_P(200, "text/html", html);
    });

    server.begin();
    Serial.println("HTTP server started");
}

void setup() {
    Serial.begin(115200);
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
    pinMode(stby, OUTPUT);
    digitalWrite(stby, HIGH);  

    if (!LittleFS.begin(true)){ 
      Serial.println("LittleFS Mount Failed");
      while (1);
    }

Serial.println("LittleFS Mounted");
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

void testDistanceTravelled() {
    const float wheelDiameter = 4.5;  // mm
    const float ppr = 7.8;            // pulses per revolution
    const float pi = 3.14159265;
    
    float wheelCircumference = pi * wheelDiameter;
    float distancePerPulse = wheelCircumference / ppr;
    
    float distanceL = encCountL * distancePerPulse;
    float distanceR = encCountR * distancePerPulse;
    
    Serial.print("Left Motor Distance: ");
    Serial.print(distanceL/90);
    Serial.println(" mm");
    
    Serial.print("Right Motor Distance: ");
    Serial.print(distanceR/90);
    Serial.println(" mm");
}

void loop() {
    // 4. Handle incoming DNS and HTTP server client requests continuously
    dnsServer.processNextRequest();
    server.handleClient();

    // Non-blocking distance logging instead of delay(2000)
    if (millis() - lastUpdate >= updateInterval) {
        testDistanceTravelled();
        lastUpdate = millis();
    }
    
    // If you plan to run the bot, you can uncomment your movement functions:
    // digitalWrite(L1, HIGH);
    // digitalWrite(L2, LOW);
    // digitalWrite(R1, HIGH);
    // digitalWrite(R2, LOW);
    // EncoderPID();
}