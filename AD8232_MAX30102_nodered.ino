#include <Wire.h>
#include <WiFi.h>
#define MQTT_MAX_PACKET_SIZE 512  // MOVED HERE - before PubSubClient
#include <PubSubClient.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#include "esp_timer.h"

// WiFi and MQTT Settings
#define WLAN_SSID "TP-Link_8602"
#define WLAN_PASS "46503190"
#define MQTT_SERVER "192.168.0.102"  // Make sure this matches your broker's IP
#define MQTT_PORT 1883
#define MQTT_USER "pi"
#define MQTT_PASSWORD "Mariuspi"

// MQTT topics (fixed typo and incorrect topic)
#define BPM_TOPIC "sensorData/bpm"
#define SPO2_TOPIC "sensorData/spo2"  
#define ECG_TOPIC "sensorData/ecg"
#define LEAD_OFF_TOPIC "sensorData/lead_off"  

// MAX30105 (I2C)
#define MAX30105_SDA 21
#define MAX30105_SCL 22

// AD8232 (ECG sensor)
#define AD8232_OUTPUT_PIN 36
#define AD8232_LO_PLUS_PIN 32
#define AD8232_LO_MINUS_PIN 33

WiFiClient espClient;
PubSubClient mqttClient(espClient);
MAX30105 sensor;

#define MAXBUFFERSIZE 100
uint32_t redBuffer[MAXBUFFERSIZE];
uint32_t irBuffer[MAXBUFFERSIZE];

float bpm_dt = 0;
float spo2_dt = 0;
volatile int ecg_value = 0;
volatile bool lead_off_plus = false;
volatile bool lead_off_minus = false;

unsigned long lastPublishTime = 0;

// ------------------ MQTT ------------------

void MQTT_connect() {
  int attempts = 0;
  while (!mqttClient.connected() && attempts < 5) { // Limit retry attempts
    attempts++;
    Serial.print("Attempting MQTT connection (attempt " + String(attempts) + ")...");
    
    // Generate a random client ID
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    // Attempt to connect with client ID and credentials
    if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("Connected to MQTT Broker");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

// ------------------ Sensor Reading ------------------

void readSensorData() {
  for (byte i = 0; i < MAXBUFFERSIZE; i++) {
    while (!sensor.available()) sensor.check();
    redBuffer[i] = sensor.getRed();
    irBuffer[i]  = sensor.getIR();
    sensor.nextSample();
  }

  int32_t heartRate;
  int8_t validHeartRate;
  int32_t spo2;
  int8_t validSpO2;

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, MAXBUFFERSIZE, redBuffer, &spo2, &validSpO2, &heartRate, &validHeartRate
  );

  bpm_dt  = validHeartRate ? heartRate : -1;
  spo2_dt = validSpO2 ? spo2 : -1;
}

void readECG() {
  ecg_value = analogRead(AD8232_OUTPUT_PIN); // Read ECG value
}

// ------------------ MQTT Publish ------------------

// Modified ECG publishing function
void publishSensorDataMQTT() {
  // Read ECG value
  ecg_value = analogRead(AD8232_OUTPUT_PIN);
  
  // Simply send the ECG value as a string
  String ecgStr = String(ecg_value);
  mqttClient.publish(ECG_TOPIC, ecgStr.c_str());
}

void publishOtherSensorDataMQTT() {
  // Read lead status before publishing
  lead_off_plus = digitalRead(AD8232_LO_PLUS_PIN);
  lead_off_minus = digitalRead(AD8232_LO_MINUS_PIN);
  
  // Publish BPM data (format as JSON for consistency)
  String bpmStr = String(bpm_dt);
  mqttClient.publish(BPM_TOPIC, bpmStr.c_str());

  // Publish SpO2 data (format as JSON for consistency)
  String spo2Str = String(spo2_dt);
  mqttClient.publish(SPO2_TOPIC, spo2Str.c_str());

  // Publish Lead Status
  String leadStatusStr;
  if (lead_off_plus && lead_off_minus) {
    leadStatusStr = "Both electrodes are disconnected (RA and LA).";
  } else if (lead_off_plus) {
    leadStatusStr = "Right arm (RA) electrode is disconnected.";
  } else if (lead_off_minus) {
    leadStatusStr = "Left arm (LA) electrode is disconnected.";
  } else {
    leadStatusStr = "Both electrodes are properly connected.";
  }
  mqttClient.publish(LEAD_OFF_TOPIC, leadStatusStr.c_str());
}

// ------------------ Timer Callback ------------------

void ecg_timer_callback(void* arg) {
  readECG();
  publishSensorDataMQTT();
}

// ------------------ Setup & Loop ------------------

void setup() {
  Serial.begin(115200);
  Wire.begin(MAX30105_SDA, MAX30105_SCL);

  // Initialize random for client ID generation
  randomSeed(micros());

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected, IP: " + WiFi.localIP().toString());

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Pinging MQTT broker...");
    IPAddress brokerIP;
    if (WiFi.hostByName(MQTT_SERVER, brokerIP)) {
      Serial.println("MQTT broker reachable at IP: " + brokerIP.toString());
    } else {
      Serial.println("MQTT broker not reachable.");
    }
  }

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  MQTT_connect();

  if (!sensor.begin()) {
    Serial.println("MAX30105 init failed!");
    while (1);
  }
  sensor.setup();
  sensor.setPulseAmplitudeRed(0x0A);
  sensor.setPulseAmplitudeIR(0x0A);

  pinMode(AD8232_LO_PLUS_PIN, INPUT);
  pinMode(AD8232_LO_MINUS_PIN, INPUT);
  pinMode(AD8232_OUTPUT_PIN, INPUT);

  const esp_timer_create_args_t ecg_timer_args = {
      .callback = &ecg_timer_callback,
      .name = "ecg_timer"
  };
  esp_timer_handle_t ecg_timer;
  esp_timer_create(&ecg_timer_args, &ecg_timer);
  esp_timer_start_periodic(ecg_timer, 10000); // Sample every 10ms (100Hz)
}

void loop() {
  if (!mqttClient.connected()) MQTT_connect();

  // Publish other data every second
  unsigned long currentMillis = millis();
  if (currentMillis - lastPublishTime >= 1000) {
    readSensorData(); // Read MAX30102 data
    publishOtherSensorDataMQTT();
    lastPublishTime = currentMillis;
  }

  mqttClient.loop(); // Handle MQTT communication
  // Removed delay to maximize sampling rate
}