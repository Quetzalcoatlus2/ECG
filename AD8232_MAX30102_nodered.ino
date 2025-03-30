#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

// WiFi and MQTT Settings
#define WLAN_SSID           "TP-Link_8602"
#define WLAN_PASS           "46503190"
#define MQTT_SERVER         "192.168.0.102"
#define MQTT_PORT           1883

const char* mqttUser = "pi";
const char* mqttPassword = "Mariuspi";

// Rewired Pins for MAX30105 (I2C)
#define MAX30105_SDA        21
#define MAX30105_SCL        22

// Rewired Pins for AD8232 (ECG sensor)
#define AD8232_OUTPUT_PIN   36
#define AD8232_LO_PLUS_PIN  32
#define AD8232_LO_MINUS_PIN 33

// Global Variables & Objects
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// MAX30105 sensor and buffers
MAX30105 sensor;
#define MAXBUFFERSIZE 100
uint32_t redBuffer[MAXBUFFERSIZE];
uint32_t irBuffer[MAXBUFFERSIZE];

// Processed sensor data
float bpm_dt = 0;
float spo2_dt = 0;

// AD8232 sensor data
volatile int ecg_value = 0;
volatile bool lead_off_plus = false;
volatile bool lead_off_minus = false;

// MQTT Topics
const char* bpm_topic = "sensorData/bpm";
const char* spo2_topic = "sensorData/spo2";
const char* ecg_topic = "sensorData/ecg";
const char* lead_off_topic = "sensorData/lead_off";

void MQTT_connect() {
  while (!mqttClient.connected()) {
    if (mqttClient.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT Broker");
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(mqttClient.state());
      delay(1000);
    }
  }
}

void readSensorData() {
  for (byte i = 0; i < MAXBUFFERSIZE; i++) {
    while (!sensor.available()) {
      sensor.check();
    }
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
  spo2_dt = validSpO2    ? spo2   : -1;
}

void readECG() {
  ecg_value = analogRead(AD8232_OUTPUT_PIN);
  lead_off_plus  = digitalRead(AD8232_LO_PLUS_PIN);
  lead_off_minus = digitalRead(AD8232_LO_MINUS_PIN);
}

void publishSensorData() {
  // Publish BPM and SpO2 as simple numeric values
  String bpmStr = String(bpm_dt);
  String spo2Str = String(spo2_dt);
  mqttClient.publish(bpm_topic, bpmStr.c_str());
  mqttClient.publish(spo2_topic, spo2Str.c_str());

  // Publish ECG data as JSON with timestamp
  String ecgStr = String("{\"x\":") + String(millis()) + String(",\"y\":") + String(ecg_value) + String("}");
  mqttClient.publish(ecg_topic, ecgStr.c_str());

  // Debug output for ECG data
  Serial.println("ECG Data Sent: " + ecgStr);

  // Publish descriptive lead-off status
  String leadOffStr;
  if (lead_off_plus && lead_off_minus) {
    leadOffStr = "Both electrodes are disconnected (RA and LA).";
  } else if (lead_off_plus) {
    leadOffStr = "Right arm (RA) electrode is disconnected.";
  } else if (lead_off_minus) {
    leadOffStr = "Left arm (LA) electrode is disconnected.";
  } else {
    leadOffStr = "Both electrodes are properly connected.";
  }
  mqttClient.publish(lead_off_topic, leadOffStr.c_str());

  // Debug output
  Serial.println("Published data to MQTT Broker:");
  Serial.println("BPM: " + bpmStr);
  Serial.println("SpO2: " + spo2Str);
  Serial.println("Lead Off: " + leadOffStr);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(MAX30105_SDA, MAX30105_SCL);

  WiFi.begin(WLAN_SSID, WLAN_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  MQTT_connect();

  Serial.print("Initializing MAX30105...");
  if (!sensor.begin()) {
    Serial.println("FAILED");
    while (1);
  }
  Serial.println("SUCCESS");
  sensor.setup();
  sensor.setPulseAmplitudeRed(0x0A);
  sensor.setPulseAmplitudeIR(0x0A);

  pinMode(AD8232_LO_PLUS_PIN, INPUT);
  pinMode(AD8232_LO_MINUS_PIN, INPUT);
}

void loop() {
  if (!mqttClient.connected()) {
    MQTT_connect();
  }

  readSensorData();
  readECG();
  publishSensorData();
  mqttClient.loop();
  delay(100);
}
