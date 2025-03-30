#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>  // Replace Adafruit MQTT library
#include "MAX30105.h"      // SparkFun MAX30105 sensor library
#include "heartRate.h"     // Heart rate calculation for MAX30105
#include "spo2_algorithm.h" // SpO2 calculation for MAX30105

// WiFi and MQTT Settings
#define WLAN_SSID           "TP-Link_8602"  // Change to your WiFi SSID
#define WLAN_PASS           "46503190"      // Change to your WiFi password
#define MQTT_SERVER         "192.168.0.102" // MQTT Broker IP (e.g., Mosquitto)
#define MQTT_PORT           1883             // MQTT Port

const char* mqttUser = "pi";  // Replace with your MQTT broker username
const char* mqttPassword = "Mariuspi";  // Replace with your MQTT broker password

// Rewired Pins for MAX30105 (I2C)
#define MAX30105_SDA        21  // I2C SDA for MAX30105
#define MAX30105_SCL        22  // I2C SCL for MAX30105

// Rewired Pins for AD8232 (ECG sensor)
#define AD8232_OUTPUT_PIN   36  // ADC pin for AD8232 ECG sensor output
#define AD8232_LO_PLUS_PIN  32  // Lead-Off Positive Pin for AD8232
#define AD8232_LO_MINUS_PIN 33  // Lead-Off Negative Pin for AD8232

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


// MAX30105 Sensor Data Processing
void readSensorData() {
  // Collect MAX30105 samples
  for (byte i = 0; i < MAXBUFFERSIZE; i++) {
    while (!sensor.available()) {
      sensor.check();  // Wait for new data
    }
    redBuffer[i] = sensor.getRed();
    irBuffer[i]  = sensor.getIR();
    sensor.nextSample();
  }

  // Calculate heart rate and SpO2
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

// AD8232 Sensor Reading
void readECG() {
  // Read the AD8232 analog ECG value
  ecg_value = analogRead(AD8232_OUTPUT_PIN);

  // Read the digital lead-off signals
  lead_off_plus  = digitalRead(AD8232_LO_PLUS_PIN);
  lead_off_minus = digitalRead(AD8232_LO_MINUS_PIN);
}

// MQTT Publish Function
void publishSensorData() {
  String bpmStr = String(bpm_dt);
  String spo2Str = String(spo2_dt);
  String ecgStr = String(ecg_value);
  String leadOffStr = String("LO+: ") + (lead_off_plus ? "Active" : "Inactive") +
                      String(", LO-: ") + (lead_off_minus ? "Active" : "Inactive");

  // Publish data to respective topics
  mqttClient.publish(bpm_topic, bpmStr.c_str());
  mqttClient.publish(spo2_topic, spo2Str.c_str());
  mqttClient.publish(ecg_topic, ecgStr.c_str());
  mqttClient.publish(lead_off_topic, leadOffStr.c_str());

  Serial.println("Published data to MQTT Broker:");
  Serial.println("BPM: " + bpmStr);
  Serial.println("SpO2: " + spo2Str);
  Serial.println("ECG: " + ecgStr);
  Serial.println("Lead Off: " + leadOffStr);
}

// Setup Function
void setup() {
  Serial.begin(115200);

  // Initialize I2C for MAX30105
  Wire.begin(MAX30105_SDA, MAX30105_SCL);

  // Connect to WiFi
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

  // Initialize MQTT connection
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  MQTT_connect();

  // Initialize MAX30105 sensor
  Serial.print("Initializing MAX30105...");
  if (!sensor.begin()) {
    Serial.println("FAILED");
    while (1);  // Halt if sensor initialization fails
  }
  Serial.println("SUCCESS");
  sensor.setup();
  sensor.setPulseAmplitudeRed(0x0A);
  sensor.setPulseAmplitudeIR(0x0A);

  // Initialize AD8232 pins
  pinMode(AD8232_LO_PLUS_PIN, INPUT);
  pinMode(AD8232_LO_MINUS_PIN, INPUT);
}

// Loop Function
void loop() {
  // Ensure MQTT connection
  if (!mqttClient.connected()) {
    MQTT_connect();
  }

  // Read sensor data
  readSensorData();
  readECG();

  // Publish collected sensor data to MQTT Broker
  publishSensorData();

  // Maintain MQTT connection
  mqttClient.loop();

  // Delay before next reading (adjust the delay for faster or slower publishing)
  delay(500);  // Delay for 0.5 seconds (can be reduced for faster publishing)
}
