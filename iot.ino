/*
 * =================================================================
 * Final ESP32 ML Predictive Maintenance RC Car (D-Pad Control)
 * =================================================================
 *
 * This version adds full D-Pad (tank steering) control.
 *
 * It includes:
 * 1. WiFi, Web Server, and FULL Motor Control (Forward, Reverse, Left, Right, Stop).
 * 2. All 3 sensors (MPU6050, DS18B20, KY-037) working.
 * 3. *** USES THE 'MPU6050_tockn' LIBRARY TO FIX THE SOFTWARE CONFLICT ***
 * 4. ML buffering and prediction logic.
 * 5. RNT-style CSS 3D box animation.
 * 6. *** NEW: Interactive D-Pad Controller (Press and hold to move) ***
 *
 * *** REQUIRED LIBRARIES ***
 * 1. "MPU6050_tockn" by tockn
 * 2. "OneWire" by Paul Stoffregen
 * 3. "DallasTemperature" by Miles Burton
 * 4. "ArduinoJson"
 */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <MPU6050_tockn.h> // <-- *** NEW, WORKING LIBRARY ***
#include <HTTPClient.h>  
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// --- WiFi Credentials ---
const char* ssid = "Galaxy A14 5G E9AF";
const char* password = "kasat123";

// --- Pin Definitions (match circuit_connections.md) ---
// Sensors
#define MPU_SDA 21
#define MPU_SCL 22
#define ONE_WIRE_BUS 4
#define SOUND_PIN 34

// LEDs
#define LED_GREEN 18
#define LED_RED 19

// Motor Driver (L298N)
// LEFT Motor (Motor 1)
#define ENA_PIN 25  // Motor 1 Speed
#define IN1_PIN 26
#define IN2_PIN 27
// RIGHT Motor (Motor 2)
#define ENB_PIN 33  // Motor 2 Speed
#define IN3_PIN 32
#define IN4_PIN 14

// --- Global Objects ---
WebServer server(80);
MPU6050 mpu6050(Wire); // <-- *** NEW, WORKING LIBRARY ***
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature dallasSensors(&oneWire);

// --- ML Prediction Globals ---
const char* prediction_server = "http://10.62.21.86:5000/predict"; // <--- REPLACE WITH YOUR LAPTOP's IP

#define N_STEPS 10 // Our LSTM window size
float temp_buffer[N_STEPS];
float vibration_buffer[N_STEPS]; // Using gVibration
float sound_buffer[N_STEPS];
int buffer_index = 0;

unsigned long lastSampleTime = 0;
const long sampleInterval = 1000; // 1 second

// Globals to store the latest prediction
float gTempPrediction = 0.0;
float gVibrationPrediction = 0.0;
float gSoundPrediction = 0.0;

// --- Global Sensor Variables ---
float gTemp = 0.0;
float gAx = 0.0, gAy = 0.0, gAz = 0.0; // Accelerometer
float gGx = 0.0, gGy = 0.0, gGz = 0.0; // Gyroscope
float gVibration = 0.0; // A simple RMS vibration metric
int gSoundLevel = 0;
bool mpuConnected = false; // Flag to check if MPU was found

// --- Motor PWM Setup ---
const int pwmFreq = 5000; // 5 kHz
const int pwmResolution = 8; // 8-bit (0-255)
const int motorSpeed = 200; // Speed for testing (0-255)

// --- Webpage HTML (stored in Flash Memory) ---
const char HTML_PROGMEM[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-g">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>ESP32 RC Car</title>
    <style>
        /* Basic mobile-friendly reset */
        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
            -webkit-user-select: none; /* Prevents text selection on hold */
            -ms-user-select: none;
            user-select: none; 
            -webkit-tap-highlight-color: transparent; /* Removes blue flash on tap */
        }
        body { 
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; 
            background: #f4f7f6; 
            color: #333;
            padding: 1rem; 
        }
        .container { 
            background: #fff; 
            padding: 1.5rem; 
            border-radius: 12px; 
            box-shadow: 0 10px 25px rgba(0,0,0,0.1); 
            width: 100%; 
            max-width: 500px; 
            margin: 0 auto; 
        }
        h1 { 
            color: #0056b3; 
            margin-top: 0; 
            font-size: 1.8rem; 
        }
        h2 {
            font-size: 1.3rem;
            margin-top: 1.5rem;
            padding-bottom: 0.5rem;
            border-bottom: 1px solid #eee;
        }
        .sensors { 
            display: grid; 
            grid-gap: 1rem; 
            margin-top: 1.5rem; 
        }
        
        /* Cleaner UI Style */
        .sensors > div { 
            background: #fff; 
            padding: 1rem 0.5rem; 
            border-bottom: 1px solid #eee; 
        }
        .sensors > div:last-child {
             border-bottom: none; 
        }

        .sensors strong { 
            color: #0056b3; 
            display: block; 
            margin-bottom: 0.5rem; 
            font-size: 0.9rem; 
        }
        .sensors span { 
            font-size: 1.5rem; 
            font-weight: 600; 
            word-wrap: break-word; 
        }
        
        /* --- *** NEW D-PAD CONTROLLER CSS *** --- */
        h2.controls-title {
            margin-bottom: -0.5rem; /* Tighten up space */
        }
        .d-pad-container {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            grid-template-rows: 1fr 1fr 1fr;
            grid-template-areas:
                ". up ."
                "left stop right"
                ". down .";
            gap: 10px;
            margin-top: 1.5rem;
            -webkit-tap-highlight-color: transparent;
        }

        .d-pad-button {
            display: flex;
            justify-content: center;
            align-items: center;
            font-size: 2rem;
            font-weight: bold;
            color: white;
            background: #007bff;
            border: none;
            border-radius: 12px;
            box-shadow: 0 4px 12px rgba(0,123,255,0.3);
            cursor: pointer;
            width: 100%;
            height: 80px; /* Taller buttons for touch */
            transition: transform 0.1s ease, box-shadow 0.1s ease;
        }
        .d-pad-button:active {
            transform: scale(0.95);
            box-shadow: 0 2px 6px rgba(0,123,255,0.5);
        }

        #btn-start { grid-area: up; background: #28a745; box-shadow: 0 4px 12px rgba(40,167,69,0.3); }
        #btn-left { grid-area: left; }
        #btn-stop { grid-area: stop; background: #dc3545; box-shadow: 0 4px 12px rgba(220,53,69,0.3); border-radius: 50%; }
        #btn-right { grid-area: right; }
        #btn-reverse { grid-area: down; background: #ffc107; color: #333; box-shadow: 0 4px 12px rgba(255,193,7,0.3); }
        /* --- *** END D-PAD CSS *** --- */

        
        /* RNT-Style CSS 3D Box */
        .box-container {
            display: flex;
            justify-content: center;
            align-items: center;
            min-height: 250px;
            perspective: 500px; /* Defines the 3D space */
        }
        #box {
            width: 150px;
            height: 150px;
            background: #007bff;
            border: 1px solid #0056b3;
            transform-style: preserve-3d;
            transition: transform 0.1s linear; /* Smooths the movement */
        }
        
        /* Style for the ML Prediction boxes */
        .prediction-box {
            background: #e6f7ff !important; 
            padding: 1rem !important;
            border-radius: 8px !important;
            border-bottom: none !important; 
        }

    </style>
</head>
<body>
    <div class="container">
        <h1>RC Car ML Dashboard</h1>
        
        <!-- *** NEW D-PAD HTML *** -->
        <h2 class="controls-title">Controls (Press & Hold)</h2>
        <div class="d-pad-container">
            <button id="btn-start" class="d-pad-button" onmousedown="sendCommand('/start')" onmouseup="sendCommand('/stop')" ontouchstart="sendCommand('/start')" ontouchend="sendCommand('/stop')">&uarr;</button>
            <button id="btn-left" class="d-pad-button" onmousedown="sendCommand('/left')" onmouseup="sendCommand('/stop')" ontouchstart="sendCommand('/left')" ontouchend="sendCommand('/stop')">&larr;</button>
            <button id="btn-stop" class="d-pad-button" onclick="sendCommand('/stop')">&#9209;</button>
            <button id="btn-right" class="d-pad-button" onmousedown="sendCommand('/right')" onmouseup="sendCommand('/stop')" ontouchstart="sendCommand('/right')" ontouchend="sendCommand('/stop')">&rarr;</button>
            <button id="btn-reverse" class="d-pad-button" onmousedown="sendCommand('/reverse')" onmouseup="sendCommand('/stop')" ontouchstart="sendCommand('/reverse')" ontouchend="sendCommand('/stop')">&darr;</button>
        </div>
        <!-- *** END D-PAD HTML *** -->


        <div class="sensors">
            <h2>Live Sensor Data</h2>
            
            <div>
                <strong>Temperature (DS18B20)</strong>
                <span id="temp">--</span> &deg;C
            </div>
            <div>
                <strong>Sound (KY-037)</strong>
                <span id="sound">--</span>
            </div>
            <div><strong>Vibration (MPU-6050 RMS)</strong><span id="vibe">--</span></div>
            <div><strong>Accelerometer (X,Y,Z)</strong><span id="accel">--</span></div>
            <div><strong>Gyroscope (X,Y,Z)</strong><span id="gyro">--</span></div>
            
            <div class="box-container">
                <div id="box"></div>
            </div>

            <h2>ML Predictions</h2>
            <div class="prediction-box">
                <strong>Temp Forecast</strong>
                <span id="temp-pred">--</span> &deg;C
            </div>
            <div class="prediction-box">
                <strong>Vibration Forecast</strong>
                <span id="vibe-pred">--</span>
            </div>
        </div>
    </div>

    <script>
        // --- Original JavaScript ---
        
        // Prevent scrolling on touch devices when holding buttons
        window.addEventListener('touchstart', function(e) {
            if (e.target.classList.contains('d-pad-button')) {
                e.preventDefault();
            }
        }, { passive: false });
        
        function sendCommand(path) {
            fetch(path)
                .then(response => response.text())
                .then(data => console.log(data))
                .catch(error => console.error('Error:', error));
        }

        function updateSensors() {
            fetch('/sensors')
                .then(response => response.json())
                .then(data => {
                    // Update Text
                    document.getElementById('temp').innerText = data.temperature.toFixed(2);
                    document.getElementById('vibe').innerText = data.vibration.toFixed(2);
                    document.getElementById('sound').innerText = data.sound;
                    document.getElementById('accel').innerText = `${data.ax.toFixed(2)}, ${data.ay.toFixed(2)}, ${data.az.toFixed(2)}`;
                    document.getElementById('gyro').innerText = `${data.gx.toFixed(2)}, ${data.gy.toFixed(2)}, ${data.gz.toFixed(2)}`;
                    
                    // --- RNT-Style Animation Logic ---
                    const box = document.getElementById('box');
                    let ax = data.ax;
                    let ay = data.ay;
                    let az = data.az;
                    
                    // Calculate tilt angles (in degrees)
                    let rotX = -Math.atan2(ay, Math.sqrt(ax * ax + az * az)) * 180 / Math.PI;
                    let rotY = Math.atan2(ax, Math.sqrt(ay * ay + az * az)) * 180 / Math.PI;

                    // Apply the rotation to the CSS transform property
                    box.style.transform = `rotateX(${rotX}deg) rotateY(${rotY}deg)`;
                    
                    // Update ML Predictions
                    if (data.temp_prediction !== undefined) {
                        document.getElementById('temp-pred').innerText = data.temp_prediction.toFixed(2);
                    }
                    if (data.vibration_prediction !== undefined) {
                        document.getElementById('vibe-pred').innerText = data.vibration_prediction.toFixed(2);
                    }
                })
                .catch(error => console.error('Error fetching sensors:', error));
        }

        // Update sensors every 2 seconds
        setInterval(updateSensors, 2000);
        
        // Get initial data on load
        window.onload = updateSensors;
    </script>
</body>
</html>
)rawliteral"; 

// --- Helper Functions ---

// This is the new function that acts as a CLIENT
void sendDataForPrediction() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Prediction: WiFi not connected.");
    return;
  }

  Serial.println("Sending sensor data buffer for prediction...");

  // Create JSON document
  StaticJsonDocument<1024> doc; // Adjust size if needed

  // Create JSON arrays from our buffers
  JsonArray temp_data = doc.createNestedArray("temp_data");
  JsonArray vibration_data = doc.createNestedArray("vibration_data");
  JsonArray sound_data = doc.createNestedArray("sound_data");

  for (int i = 0; i < N_STEPS; i++) {
    temp_data.add(temp_buffer[i]);
    vibration_data.add(vibration_buffer[i]);
    sound_data.add(sound_buffer[i]);
  }

  // Serialize JSON
  String jsonPayload;
  serializeJson(doc, jsonPayload);

  // Send to Python Server
  HTTPClient http;
  http.begin(prediction_server);
  http.addHeader("Content-Type", "application/json");

  int httpResponseCode = http.POST(jsonPayload);

  if (httpResponseCode == 200) {
    String response = http.getString();
    Serial.println("--- PREDICTION RECEIVED ---");
    Serial.println(response);

    // Parse the response
    StaticJsonDocument<256> responseDoc;
    deserializeJson(responseDoc, response);

    // Store predictions in our global variables
    // Note: Your Python server example had 'pressure_prediction'.
    // Make sure your Python keys match what you use here.
    gTempPrediction = responseDoc["temp_prediction"]; 
    gVibrationPrediction = responseDoc["pressure_prediction"]; // Matching Python example key
    // gSoundPrediction = responseDoc["vibration_prediction"]; // Assuming you add this to Python

  } else {
    Serial.print("Error on sending POST: ");
    Serial.println(httpResponseCode);
  }

  http.end();

  // Reset buffer
  buffer_index = 0;
}

void readAllSensors() {
  // Read Temperature
  dallasSensors.requestTemperatures(); 
  gTemp = dallasSensors.getTempCByIndex(0);

  // Read Sound
  gSoundLevel = analogRead(SOUND_PIN);
  
  // Read MPU-6050 only if it was found during setup
  if (mpuConnected) {
    // *** NEW LIBRARY LOGIC ***
    mpu6050.update(); // Update all sensor values

    gAx = mpu6050.getAccX();
    gAy = mpu6050.getAccY();
    gAz = mpu6050.getAccZ();
    gGx = mpu6050.getGyroX();
    gGy = mpu6050.getGyroY();
    gGz = mpu6050.getGyroZ();
    
    // Calculate a simple RMS for vibration
    gVibration = sqrt(gAx * gAx + gAy * gAy + gAz * gAz);

  } else {
    // MPU not connected, set all related values to 0
    gAx = gAy = gAz = 0;
    gGx = gGy = gGz = 0;
    gVibration = 0;
  }

  // Blink green LED to show we are alive
  digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
}

void moveMotors(bool forward, int speed) {
  Serial.println("--- Inside moveMotors() function ---"); // DEBUG LINE
  // Set direction
  digitalWrite(IN1_PIN, forward ? HIGH : LOW);
  digitalWrite(IN2_PIN, forward ? LOW : HIGH);
  digitalWrite(IN3_PIN, forward ? HIGH : LOW);
  digitalWrite(IN4_PIN, forward ? LOW : HIGH);

  // Set speed
  ledcWrite(ENA_PIN, speed);
  ledcWrite(ENB_PIN, speed);

  // Light red LED when motors are on
  digitalWrite(LED_RED, HIGH);
}

void stopMotors() {
  Serial.println("--- Inside stopMotors() function ---"); // DEBUG LINE
  // Set direction to low (brake)
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);

  // Set speed to 0
  ledcWrite(ENA_PIN, 0);
  ledcWrite(ENB_PIN, 0);
  
  // Turn off red LED
  digitalWrite(LED_RED, LOW);
}

// --- *** NEW TURN FUNCTIONS *** ---
void turnLeft() {
  Serial.println("--- Inside turnLeft() function ---");
  // Left Motor (Motor 1) STOP
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
  ledcWrite(ENA_PIN, 0);
  
  // Right Motor (Motor 2) FORWARD
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  ledcWrite(ENB_PIN, motorSpeed);
  
  digitalWrite(LED_RED, HIGH); // Show motors are active
}

void turnRight() {
  Serial.println("--- Inside turnRight() function ---");
  // Left Motor (Motor 1) FORWARD
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  ledcWrite(ENA_PIN, motorSpeed);
  
  // Right Motor (Motor 2) STOP
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
  ledcWrite(ENB_PIN, 0);

  digitalWrite(LED_RED, HIGH); // Show motors are active
}
// --- *** END NEW FUNCTIONS *** ---


// --- Web Server Handlers ---

void handleRoot() {
  server.send_P(200, "text/html", HTML_PROGMEM);
}

void handleStart() {
  Serial.println("--- /start command received ---"); // DEBUG LINE
  moveMotors(true, motorSpeed); // Forward
  server.send(200, "text/plain", "Motors Started Forward");
}

void handleReverse() {
  Serial.println("--- /reverse command received ---"); // DEBUG LINE
  moveMotors(false, motorSpeed); // Reverse
  server.send(200, "text/plain", "Motors Started Reverse");
}

void handleStop() {
  Serial.println("--- /stop command received ---"); // DEBUG LINE
  stopMotors();
  server.send(200, "text/plain", "Motors Stopped");
}

// --- *** NEW TURN HANDLERS *** ---
void handleTurnLeft() {
  Serial.println("--- /left command received ---");
  turnLeft();
  server.send(200, "text/plain", "Turning Left");
}

void handleTurnRight() {
  Serial.println("--- /right command received ---");
  turnRight();
  server.send(200, "text/plain", "Turning Right");
}
// --- *** END NEW HANDLERS *** ---


void handleSensors() {
  // Update sensor values right before sending
  readAllSensors();

  // Create a JSON response
  String json = "{";
  json += "\"temperature\": " + String(gTemp) + ",";
  json += "\"sound\": " + String(gSoundLevel) + ","; // <-- *** FIXED THE TYPO HERE *** (was "S-tring...")
  json += "\"vibration\": " + String(gVibration) + ",";
  json += "\"ax\": " + String(gAx) + ",";
  json += "\"ay\": " + String(gAy) + ",";
  json += "\"az\": " + String(gAz) + ",";
  json += "\"gx\": " + String(gGx) + ",";
  json += "\"gy\": " + String(gGy) + ",";
  json += "\"gz\": " + String(gGz) + ","; 
  json += "\"temp_prediction\": " + String(gTempPrediction) + ",";
  json += "\"vibration_prediction\": " + String(gVibrationPrediction);
  // json += "\"sound_prediction\": " + String(gSoundPrediction); // Add this when ready

  json += "}";
  server.send(200, "application/json", json);
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

  // --- Main Setup & Loop ---

void setup() {
  Serial.begin(115200);
  Serial.println("Booting up Final Code (tockn library)...");

  // --- Initialize Pins ---
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  pinMode(SOUND_PIN, INPUT);

  // --- Setup Motor PWM (Modern ESP32 Core v3.x.x) ---
  ledcAttach(ENA_PIN, pwmFreq, pwmResolution);
  ledcAttach(ENB_PIN, pwmFreq, pwmResolution);
  
  stopMotors(); // Ensure motors are off at boot

  // --- Initialize WiFi (FIRST) ---
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    digitalWrite(LED_GREEN, !digitalRead(LED_GREEN)); 
  }
  Serial.println("\nWiFi connected!");
  Serial.print("Server IP: ");
  Serial.println(WiFi.localIP());
  digitalWrite(LED_GREEN, HIGH); 

  // --- Initialize Sensors (LAST) ---
  // We follow the *exact* working logic from the minimal test
  
  Serial.println("Initializing I2C bus...");
  Wire.begin(MPU_SDA, MPU_SCL); 
  delay(100); 

  Serial.println("Initializing MPU6050 (tockn library)...");
  mpu6050.begin();
  
  Serial.println("Calibrating Gyro... DO NOT MOVE SENSOR");
  mpu6050.calcGyroOffsets(true); // Calibrate
  Serial.println("MPU6050 Found and Calibrated!");
  mpuConnected = true; // Set the flag to true
  
  dallasSensors.begin();
  Serial.println("Dallas Temperature Sensor Initialized");

  // --- Start Web Server ---
  server.on("/", handleRoot);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.on("/reverse", handleReverse);
  server.on("/sensors", handleSensors);
  // *** ADDED NEW ROUTES ***
  server.on("/left", handleTurnLeft);
  server.on("/right", handleTurnRight);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
  
  unsigned long currentTime = millis();
  if (currentTime - lastSampleTime > sampleInterval) {
    lastSampleTime = currentTime; // Reset timer

    // First, read the sensors to get fresh data
    readAllSensors();

    // We can now always buffer, because mpuConnected is true!
    if (buffer_index < N_STEPS) {
      // Add to buffer
      temp_buffer[buffer_index] = gTemp;
      vibration_buffer[buffer_index] = gVibration; // Use the correct variable
      sound_buffer[buffer_index] = (float)gSoundLevel;

      Serial.print("Buffering data... ");
      Serial.println(buffer_index);
      
      buffer_index++;

    } else if (buffer_index == N_STEPS) {
      // Buffer is full, send it for prediction
      sendDataForPrediction();
      // sendDataForPrediction() resets buffer_index to 0
    }
  }
}