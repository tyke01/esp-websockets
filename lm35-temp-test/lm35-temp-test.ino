#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>

// WiFi credentials - CHANGE THESE TO YOUR NETWORK
const char* ssid = "Victors Machine";
const char* password = "COOL12345";

// Create WebSocket server on port 81
WebSocketsServer webSocket = WebSocketsServer(81);

// LM35 temperature sensor constants
#define ADC_VREF_mV 3300.0
#define ADC_RESOLUTION 4096.0
#define PIN_LM35 32

// Calibration offset
#define TEMP_OFFSET -25.0

// LED pins
#define LED1_PIN 16  // ≥35°C
#define LED2_PIN 2   // ≥40°C
#define LED3_PIN 15  // ≥45°C
#define LED4_PIN 4   // ≥50°C

// Temperature thresholds
#define TEMP_THRESHOLD_1 35.0
#define TEMP_THRESHOLD_2 40.0
#define TEMP_THRESHOLD_3 45.0
#define TEMP_THRESHOLD_4 50.0

// Vibration thresholds
#define VIBRATION_THRESHOLD_1 12.0
#define VIBRATION_THRESHOLD_2 16.0
#define VIBRATION_THRESHOLD_3 20.0
#define VIBRATION_THRESHOLD_4 24.0

Adafruit_MPU6050 mpu;

// Keep track of connected clients
int connectedClients = 0;

unsigned long lastSensorRead = 0;
const unsigned long sensorInterval = 100;  // 100ms interval

// Store latest sensor data
struct SensorData {
  float lm35Temp;
  float tempF;
  float mpuTemp;
  float vibration;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  bool led1, led2, led3, led4;
  unsigned long timestamp;
} latestData;

void webSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[WebSocket] Client #%u Disconnected\n", num);
      connectedClients--;
      break;

    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[WebSocket] Client #%u Connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
        connectedClients++;

        // Send current sensor data to newly connected client
        String sensorData = createSensorJSON();
        webSocket.sendTXT(num, sensorData);
        break;
      }

    case WStype_TEXT:
      {
        Serial.printf("[WebSocket] Received text from client #%u: %s\n", num, payload);

        // Handle client requests
        String message = String((char*)payload);
        if (message == "getSensorData") {
          String sensorData = createSensorJSON();
          webSocket.sendTXT(num, sensorData);
        }
        break;
      }

    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  // Initialize LED pins
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);

  // Turn off all LEDs initially
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
  digitalWrite(LED4_PIN, LOW);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("MPU6050 initialization failed!");
    while (1) delay(100);
  }

  Serial.println("MPU6050 initialized successfully");

  // Configure sensor settings
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  // Initialize WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 20) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi connected! IP address: ");
    Serial.println(WiFi.localIP());

    // Start WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);

    Serial.println("WebSocket server started on port 81");
    Serial.println("Connect to: ws://" + WiFi.localIP().toString() + ":81");
  } else {
    Serial.println();
    Serial.println("WiFi connection failed! Running in offline mode.");
  }

  delay(100);
}

void loop() {
  // Handle WebSocket events
  if (WiFi.status() == WL_CONNECTED) {
    webSocket.loop();
  }

  // Read sensors at specified interval
  if (millis() - lastSensorRead >= sensorInterval) {
    lastSensorRead = millis();

    // Read LM35 temperature
    int adcVal = analogRead(PIN_LM35);
    float milliVolt = adcVal * (ADC_VREF_mV / ADC_RESOLUTION);
    float lm35Temp = (milliVolt / 10) + TEMP_OFFSET;
    float tempF = lm35Temp * 9 / 5 + 32;

    // Read MPU6050 data
    sensors_event_t accel, gyro, mpuTemp;
    mpu.getEvent(&accel, &gyro, &mpuTemp);

    // Calculate total acceleration magnitude
    float accelMagnitude = sqrt(pow(accel.acceleration.x, 2) + pow(accel.acceleration.y, 2) + pow(accel.acceleration.z, 2));

    // Control LEDs
    controlLEDs(lm35Temp, accelMagnitude);

    // Update stored sensor data
    latestData.lm35Temp = lm35Temp;
    latestData.tempF = tempF;
    latestData.mpuTemp = mpuTemp.temperature;
    latestData.vibration = accelMagnitude;
    latestData.accelX = accel.acceleration.x;
    latestData.accelY = accel.acceleration.y;
    latestData.accelZ = accel.acceleration.z;
    latestData.gyroX = gyro.gyro.x;
    latestData.gyroY = gyro.gyro.y;
    latestData.gyroZ = gyro.gyro.z;
    latestData.led1 = (lm35Temp >= TEMP_THRESHOLD_1) || (accelMagnitude >= VIBRATION_THRESHOLD_1);
    latestData.led2 = (lm35Temp >= TEMP_THRESHOLD_2) || (accelMagnitude >= VIBRATION_THRESHOLD_2);
    latestData.led3 = (lm35Temp >= TEMP_THRESHOLD_3) || (accelMagnitude >= VIBRATION_THRESHOLD_3);
    latestData.led4 = (lm35Temp >= TEMP_THRESHOLD_4) || (accelMagnitude >= VIBRATION_THRESHOLD_4);
    latestData.timestamp = millis();

    // Print to Serial
    Serial.printf("Temp: %.2f°C | Vibration: %.2f m/s² | Clients: %d\n",
                  lm35Temp, accelMagnitude, connectedClients);

    // Send data to all connected WebSocket clients every second
    static unsigned long lastBroadcast = 0;
    if (millis() - lastBroadcast >= 1000 && connectedClients > 0) {
      lastBroadcast = millis();
      String sensorData = createSensorJSON();
      webSocket.broadcastTXT(sensorData);
    }
  }
}

void controlLEDs(float temperature, float vibration) {
  bool tempLED1 = temperature >= TEMP_THRESHOLD_1;
  bool tempLED2 = temperature >= TEMP_THRESHOLD_2;
  bool tempLED3 = temperature >= TEMP_THRESHOLD_3;
  bool tempLED4 = temperature >= TEMP_THRESHOLD_4;

  bool vibLED1 = vibration >= VIBRATION_THRESHOLD_1;
  bool vibLED2 = vibration >= VIBRATION_THRESHOLD_2;
  bool vibLED3 = vibration >= VIBRATION_THRESHOLD_3;
  bool vibLED4 = vibration >= VIBRATION_THRESHOLD_4;

  digitalWrite(LED1_PIN, (tempLED1 || vibLED1) ? HIGH : LOW);
  digitalWrite(LED2_PIN, (tempLED2 || vibLED2) ? HIGH : LOW);
  digitalWrite(LED3_PIN, (tempLED3 || vibLED3) ? HIGH : LOW);
  digitalWrite(LED4_PIN, (tempLED4 || vibLED4) ? HIGH : LOW);
}

String createSensorJSON() {
  DynamicJsonDocument doc(1024);

  doc["timestamp"] = latestData.timestamp;
  doc["lm35_temp_c"] = round(latestData.lm35Temp * 100.0) / 100.0;
  doc["lm35_temp_f"] = round(latestData.tempF * 100.0) / 100.0;
  doc["mpu_temp_c"] = round(latestData.mpuTemp * 100.0) / 100.0;
  doc["vibration"] = round(latestData.vibration * 100.0) / 100.0;

  // Acceleration data
  JsonObject acceleration = doc.createNestedObject("acceleration");
  acceleration["x"] = round(latestData.accelX * 100.0) / 100.0;
  acceleration["y"] = round(latestData.accelY * 100.0) / 100.0;
  acceleration["z"] = round(latestData.accelZ * 100.0) / 100.0;

  // Gyroscope data
  JsonObject rotation = doc.createNestedObject("rotation");
  rotation["x"] = round(latestData.gyroX * 100.0) / 100.0;
  rotation["y"] = round(latestData.gyroY * 100.0) / 100.0;
  rotation["z"] = round(latestData.gyroZ * 100.0) / 100.0;

  // LED status
  JsonObject leds = doc.createNestedObject("leds");
  leds["led1"] = latestData.led1;
  leds["led2"] = latestData.led2;
  leds["led3"] = latestData.led3;
  leds["led4"] = latestData.led4;

  // Alert status
  JsonObject alerts = doc.createNestedObject("alerts");
  alerts["temp_alert"] = latestData.lm35Temp >= TEMP_THRESHOLD_1;
  alerts["vibration_alert"] = latestData.vibration >= VIBRATION_THRESHOLD_1;

  // Connection info
  doc["connected_clients"] = connectedClients;
  doc["wifi_connected"] = WiFi.status() == WL_CONNECTED;

  String output;
  serializeJson(doc, output);
  return output;
}