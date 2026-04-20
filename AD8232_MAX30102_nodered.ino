#include <Wire.h> // I2C communication library (required for the MAX30102 sensor)
#include <WiFi.h> // Library for managing the WiFi connection
#define MQTT_MAX_PACKET_SIZE 512 // Increases the maximum MQTT packet size to allow larger payloads
#include <PubSubClient.h> // Library for the MQTT client
#include "MAX30105.h" // SparkFun library for pulse and SpO2 sensor (also compatible with MAX30102)
#include "heartRate.h" // Header for heart-rate calculation algorithm
#include "spo2_algorithm.h" // Header for SpO2 calculation algorithm
#include "esp_timer.h" // Library for high-precision timers on ESP32
#include <math.h> // Math library, required for PI constant

// ADC reference voltage
const float ADC_REFERENCE_VOLTAGE = 0.0033; // ADC reference voltage in Volts (3.3V); using 0.0033 for correct Grafana representation
const float ADC_MAX_VALUE = 4095.0; // Maximum value for 12-bit ADC (2^12 - 1)

// ECG filter parameters
const float SAMPLING_FREQUENCY = 200.0; // Frequency used to read the ECG signal (Hz)
const float SAMPLING_PERIOD = 1.0 / SAMPLING_FREQUENCY; // Sampling period, calculated from frequency
const float LPF_CUTOFF_FREQUENCY = 25.0; // Cutoff frequency for the low-pass filter (removes high-frequency noise)
const float LPF_RC = 1.0 / (2.0 * PI * LPF_CUTOFF_FREQUENCY); // Time constant for LPF
const float LPF_ALPHA = SAMPLING_PERIOD / (LPF_RC + SAMPLING_PERIOD); // Smoothing factor for LPF
volatile float previous_lpf_output = 0.0; // Stores previous LPF output value

const float HPF_CUTOFF_FREQUENCY = 0.5; // Cutoff frequency for the high-pass filter (removes DC component)
const float HPF_RC = 1.0 / (2.0 * PI * HPF_CUTOFF_FREQUENCY); // Time constant for HPF
const float HPF_ALPHA = HPF_RC / (HPF_RC + SAMPLING_PERIOD); // Smoothing factor for HPF
volatile float previous_hpf_output = 0.0; // Stores previous HPF output value
volatile float previous_hpf_input = 0.0; // Stores previous HPF input value

volatile float current_filtered_ecg_mv = 0.0; // Current filtered ECG signal value, in millivolts
volatile bool ecg_ready_to_send = false; // Flag indicating whether a new ECG value is ready to send

#define ECG_AVERAGE_SAMPLE_COUNT 8 // Number of ECG samples used for moving average
float ecg_history[ECG_AVERAGE_SAMPLE_COUNT]; // Array to store latest ECG values
int ecg_history_index = 0; // Current index in ECG history array
bool is_ecg_history_full = false; // Flag indicating whether history array is full
float ecg_average_for_send = 0.0; // Average ECG value to be sent via MQTT

#define BPM_AVERAGE_SAMPLE_COUNT 5 // Number of BPM samples used for moving average
float bpm_history[BPM_AVERAGE_SAMPLE_COUNT]; // Array to store latest BPM values
int bpm_history_index = 0; // Current index in BPM history array
bool is_bpm_history_full = false; // Flag indicating whether history array is full
float bpm_average_for_send = 0.0; // Average BPM value to be sent

#define FINGER_IR_THRESHOLD 50000 // Minimum IR signal value to consider that a finger is on the sensor
#define MIN_BPM 30.0 // Minimum acceptable BPM value
#define MAX_BPM 220.0 // Maximum acceptable BPM value
#define MIN_SPO2 70.0 // Minimum acceptable SpO2 value
#define MAX_SPO2 100.0 // Maximum acceptable SpO2 value

#define PIN_SDA_MAX30102 21 // I2C SDA pin for MAX30102 sensor
#define PIN_SCL_MAX30102 22 // I2C SCL pin for MAX30102 sensor

#define PIN_ECG 36 // ADC pin for reading ECG signal
#define PIN_LO_PLUS 32 // Pin for detecting positive electrode contact, LA (Leads-Off+)
#define PIN_LO_MINUS 33 // Pin for detecting negative electrode contact, RA (Leads-Off-)

#define PIN_BUTTON 14 // Pin for panic button
#define PIN_BUZZER 26 // Pin for alarm buzzer
#define BUZZER_FREQUENCY 2000 // Frequency of buzzer sound (Hz)

const char* WIFI_SSID = "Device"; // WiFi network name
const char* WIFI_PASSWORD = "46503190"; // WiFi network password
const char* SERVER_MQTT = "192.168.220.144"; // IP address of MQTT broker (Raspberry Pi)
const int PORT_MQTT = 1883; // Standard MQTT port
const char* MQTT_USERNAME = "pi"; // Username for MQTT authentication
const char* MQTT_PASSWORD = "Mariuspi"; // Password for MQTT authentication

// MQTT TOPICS for Node-RED
#define TOPIC_BPM "sensorData/bpm"
#define TOPIC_SPO2 "sensorData/spo2"
#define TOPIC_ECG "sensorData/ecg"
#define TOPIC_LEAD_OFF "sensorData/lead_off"
#define TOPIC_BUTTON "sensorData/button"

// Buffer for raw MAX30102 data
#define MAX30102_SAMPLE_COUNT 50 // Number of samples collected from MAX30102 before processing
uint32_t buffer_ir[MAX30102_SAMPLE_COUNT]; // Buffer for infrared (IR) values
uint32_t red_buffer[MAX30102_SAMPLE_COUNT]; // Buffer for red LED values
int32_t spo2; // Variable to store calculated SpO2 value
int8_t spo2_valid; // Flag indicating whether SpO2 value is valid
int32_t pulse; // Variable to store calculated pulse value
int8_t is_pulse_valid; // Flag indicating whether pulse value is valid

// Global state for non-blocking MAX30102 collection
volatile bool is_max_collection_in_progress = false; // Flag indicating whether data is being collected from MAX30102
volatile int collected_sample_count = 0; // Counter for number of collected samples

MAX30105 pulse_sensor; // Object for pulse sensor (MAX30105 class from library)
float current_bpm = 0.0; // Current BPM value, before averaging
float current_spo2 = 0.0; // Current SpO2 value
volatile bool lead_off_plus = false; // Current state of positive electrode
volatile bool lead_off_minus = false; // Current state of negative electrode

WiFiClient wifi_client; // WiFi client object
PubSubClient mqtt_client(wifi_client); // MQTT client object using WiFi client

unsigned long last_send_time = 0; // Stores timestamp of last sensor data send (BPM/SpO2)
const unsigned long other_sensor_interval = 5000; // Interval in milliseconds for sending data (5 seconds)

unsigned long last_debounce_time = 0; // Stores timestamp of last button state change
const unsigned long debounce_ms = 50; // Debounce time for button (avoids multiple reads)
int previous_button_state = HIGH; // Previous button state (HIGH = not pressed)
int current_button_state = HIGH; // Current button state
bool is_button_alarm_active = false; // Flag indicating whether button alarm is active

esp_timer_handle_t ecg_timer; // Handle (identifier) for ECG high-precision timer

// Function declarations (prototypes)
void connect_MQTT();
void process_max30102();
void send_ECG_MQTT();
void send_non_ECG_MQTT();
void callback_timer_ecg(void* arg);

// Constants for ECG moving average
const int ecgAverageValueCount = 20;
float ecgValues[ecgAverageValueCount];

void connect_MQTT() {
  Serial.print("Attempting MQTT connection...");
  String client_id = "ESP32Client-"; // Creates a unique client ID
  client_id += String(random(0xffff), HEX);
  int attempts = 0;
  while (!mqtt_client.connected() && attempts < 5) { // Attempts to connect 5 times
    Serial.print("\nAttempt ");
    Serial.print(attempts + 1);
    Serial.print(" to MQTT broker: ");
    Serial.println(SERVER_MQTT);
    if (mqtt_client.connect(client_id.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) { // Connect with ID, username, and password
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(mqtt_client.state()); // Displays error code
      Serial.println(" retry in 5 seconds");
      delay(5000); // Waits 5 seconds before retrying
    }
    attempts++;
  }
  if (!mqtt_client.connected()) {
    Serial.println("Could not connect to MQTT broker after multiple attempts.");
  }
}

// Non-blocking MAX30102 processing
void process_max30102() {
    if (!is_max_collection_in_progress) { // Runs only if a collection is in progress
        return;
    }
    if (collected_sample_count < MAX30102_SAMPLE_COUNT) { // Checks whether buffer is not full
        if (pulse_sensor.available()) { // Checks whether new data exists in sensor FIFO
            red_buffer[collected_sample_count] = pulse_sensor.getFIFORed(); // Reads Red value
            buffer_ir[collected_sample_count] = pulse_sensor.getFIFOIR(); // Reads IR value
            pulse_sensor.nextSample(); // Moves to next FIFO sample
            collected_sample_count++; // Increments sample counter

            if (collected_sample_count >= MAX30102_SAMPLE_COUNT) { // If buffer is full
                // Calculates SpO2 and pulse using buffer data
                maxim_heart_rate_and_oxygen_saturation(buffer_ir, MAX30102_SAMPLE_COUNT, red_buffer, &spo2, &spo2_valid, &pulse, &is_pulse_valid);

                if (spo2_valid && spo2 >= MIN_SPO2 && spo2 <= MAX_SPO2) { // Validates SpO2
                    current_spo2 = (float)spo2;
                } else {
                    current_spo2 = 0.0; // Sets to 0 if invalid
                }

                if (is_pulse_valid && pulse >= MIN_BPM && pulse <= MAX_BPM) { // Validates pulse
                    current_bpm = (float)pulse;
                } else {
                    current_bpm = 0.0; // Sets to 0 if invalid
                }
                is_max_collection_in_progress = false; // Stops collection flag
            }
        }
    }
}

void send_ECG_MQTT() {
  if (!mqtt_client.connected()) { // Checks MQTT connection
    return;
  }
  float ecg_sample = current_filtered_ecg_mv; // Gets latest filtered ECG value
  ecg_history[ecg_history_index] = ecg_sample; // Adds value to average history
  ecg_history_index = (ecg_history_index + 1) % ECG_AVERAGE_SAMPLE_COUNT; // Updates circular index
  if (!is_ecg_history_full && ecg_history_index == 0) {
    is_ecg_history_full = true; // Marks history as full
  }

  if (is_ecg_history_full) { // Calculates average if history is full
    float sum = 0;
    for (int i = 0; i < ECG_AVERAGE_SAMPLE_COUNT; i++) {
      sum += ecg_history[i];
    }
    ecg_average_for_send = sum / ECG_AVERAGE_SAMPLE_COUNT;
  } else { // Otherwise, use current value
    ecg_average_for_send = ecg_sample;
  }

  char payload_ecg[20]; // Buffer for MQTT payload
  dtostrf(ecg_average_for_send, 8, 5, payload_ecg); // Converts float value to string with 5 decimals
  payload_ecg[19] = '\0'; // Ensures proper string termination
  if (!mqtt_client.publish(TOPIC_ECG, payload_ecg)) { // Publishes on ECG topic
    Serial.println("ECG MQTT send failed");
  }
}

void send_non_ECG_MQTT() {
  if (!mqtt_client.connected()) { // Checks MQTT connection
    return;
  }
  char payload_numeric[20]; // Buffer for numeric payloads

  // Update and calculate BPM moving average
  bpm_history[bpm_history_index] = current_bpm; // Adds current BPM value to history
  bpm_history_index = (bpm_history_index + 1) % BPM_AVERAGE_SAMPLE_COUNT; // Updates circular index
  if (!is_bpm_history_full && bpm_history_index == 0) {
    is_bpm_history_full = true; // Marks history as full
  }

  float bpm_sum_calc = 0.0;
  int count_samples_for_avg = 0;

  if (is_bpm_history_full) { // Calculates average if history is full
    for (int i = 0; i < BPM_AVERAGE_SAMPLE_COUNT; i++) {
      bpm_sum_calc += bpm_history[i];
    }
    count_samples_for_avg = BPM_AVERAGE_SAMPLE_COUNT;
  } else { // Otherwise, calculates average on available samples
    for (int i = 0; i < bpm_history_index; i++) {
      bpm_sum_calc += bpm_history[i];
    }
    count_samples_for_avg = bpm_history_index;
  }

  if (count_samples_for_avg > 0) {
    bpm_average_for_send = bpm_sum_calc / count_samples_for_avg;
  } else {
    bpm_average_for_send = current_bpm; // Uses current value if no samples are available for average
  }

  // BPM
  sprintf(payload_numeric, "%.2f", bpm_average_for_send); // Formats BPM value with 2 decimals
  mqtt_client.publish(TOPIC_BPM, payload_numeric); // Publishes on BPM topic

  // SpO2
  sprintf(payload_numeric, "%.2f", current_spo2); // Formats SpO2 value with 2 decimals
  mqtt_client.publish(TOPIC_SPO2, payload_numeric); // Publishes on SpO2 topic

  // Electrode status (Lead status) in JSON format
  String lead_status_description = "";
  int lead_status_code = 0;
  if (lead_off_plus && lead_off_minus) {
    lead_status_description = "Both electrodes are disconnected.";
    lead_status_code = 3;
  } else if (lead_off_plus) {
    lead_status_description = "LA electrode is disconnected.";
    lead_status_code = 1;
  } else if (lead_off_minus) {
    lead_status_description = "RA electrode is disconnected.";
    lead_status_code = 2;
  } else {
    lead_status_description = "Both electrodes are connected.";
    lead_status_code = 0;
  }
  String lead_status_payload_json = "{\"description\":\"" + lead_status_description + "\", \"code\":" + String(lead_status_code) + "}"; // Creates JSON payload
  mqtt_client.publish(TOPIC_LEAD_OFF, lead_status_payload_json.c_str()); // Publishes on electrode topic
}

// Timer callback function, runs at each ECG sample
void callback_timer_ecg(void* arg) {
  int adc_value = analogRead(PIN_ECG); // Reads raw value from ADC
  float voltage = (adc_value / ADC_MAX_VALUE) * ADC_REFERENCE_VOLTAGE; // Converts to voltage
  float iesire_ftj = LPF_ALPHA * voltage + (1 - LPF_ALPHA) * previous_lpf_output; // Applies low-pass filter
  previous_lpf_output = iesire_ftj; // Saves filter state
  float iesire_fts = HPF_ALPHA * (previous_hpf_output + iesire_ftj - previous_hpf_input); // Applies high-pass filter
  previous_hpf_input = iesire_ftj; // Saves filter state
  previous_hpf_output = iesire_fts; // Saves filter state
  current_filtered_ecg_mv = iesire_fts * 1000.0; // Converts filtered voltage to millivolts
  ecg_ready_to_send = true; // Sets flag for sending
  lead_off_plus = digitalRead(PIN_LO_PLUS); // Reads + electrode state
  lead_off_minus = digitalRead(PIN_LO_MINUS); // Reads - electrode state
}

void setup() {
  Serial.begin(115200); // Initializes serial communication
  Wire.begin(PIN_SDA_MAX30102, PIN_SCL_MAX30102); // Initializes I2C communication on specified pins

  if (!pulse_sensor.begin(Wire, I2C_SPEED_FAST)) { // Initializes MAX30102 sensor
    Serial.println("MAX30102 was not found. Check connections/power supply.");
    while (1); // Stops execution if sensor is not found
  }
  Serial.println("MAX30102 initialized.");

  // MAX30102 configuration: LED power, sample average, mode (Red+IR), sample rate, pulse width, ADC range
  pulse_sensor.setup(0x1F, 4, 2, 100, 411, 4096);

  randomSeed(micros()); // Initializes random number generator
  pinMode(PIN_BUTTON, INPUT_PULLUP); // Configures button pin as input with pull-up resistor
  pinMode(PIN_BUZZER, OUTPUT); // Configures buzzer pin as output
  noTone(PIN_BUZZER); // Stops buzzer at startup

  for(int i=0; i<ECG_AVERAGE_SAMPLE_COUNT; ++i) ecg_history[i] = 0.0; // Initializes ECG history with 0
  for(int i=0; i<BPM_AVERAGE_SAMPLE_COUNT; ++i) bpm_history[i] = 0.0; // Initializes BPM history with 0

  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // Starts WiFi connection
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) { // Waits for connection
    delay(500);
    Serial.print(".");
    wifi_attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP()); // Displays obtained IP address
  } else {
    Serial.println("\nWiFi connection failed. Stopping.");
    while(true) { delay(1000); } // Stops execution if WiFi connection cannot be established
  }

  mqtt_client.setServer(SERVER_MQTT, PORT_MQTT); // Sets MQTT server
  connect_MQTT(); // Connects to MQTT broker

  pinMode(PIN_ECG, INPUT); // Configures ECG pin as input
  pinMode(PIN_LO_PLUS, INPUT); // Configures LO+ pin as input
  pinMode(PIN_LO_MINUS, INPUT); // Configures LO- pin as input
  analogSetPinAttenuation(PIN_ECG, ADC_11db); // Sets attenuation to allow reading voltages up to 3.3V

  // Initialize filters with a first reading to avoid startup jump
  int initial_value = analogRead(PIN_ECG);
  float initial_voltage = (initial_value / ADC_MAX_VALUE) * ADC_REFERENCE_VOLTAGE;
  previous_lpf_output = initial_voltage;
  previous_hpf_input = initial_voltage;
  previous_hpf_output = 0.0;

  // Configure and start timer for ECG sampling
  const esp_timer_create_args_t args_timer_ecg = {
      .callback = &callback_timer_ecg, // Function to be called by timer
      .name = "ecg_timer" // Timer name
  };
  esp_timer_create(&args_timer_ecg, &ecg_timer); // Creates timer
  esp_timer_start_periodic(ecg_timer, (uint64_t)(SAMPLING_PERIOD * 1000000)); // Starts periodic timer
}

void loop() {
  pulse_sensor.check(); // Checks MAX30102 sensor state (e.g., reads FIFO data)
  process_max30102(); // Processes collected MAX30102 data, if needed

  if (WiFi.status() != WL_CONNECTED) { // Checks whether WiFi connection is still active
    Serial.println("WiFi disconnected. Reconnecting...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // Attempts reconnection
    int wifi_attempts = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) {
      delay(500);
      Serial.print(".");
      wifi_attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi reconectat.");
      connect_MQTT(); // Reconnect to MQTT after WiFi returns
    } else {
      Serial.println("\nWiFi reconnection failed.");
      delay(5000);
      return; // Exits loop to retry later
    }
  }

  if (!mqtt_client.connected()) { // Checks whether MQTT client is still connected
    connect_MQTT(); // Reconnects if needed
  }
  mqtt_client.loop(); // Keeps MQTT connection alive and processes incoming messages

  if (ecg_ready_to_send) { // If a new ECG value is available
    send_ECG_MQTT(); // Sends value via MQTT
    ecg_ready_to_send = false; // Resets flag
  }

  // Logic for panic button with debounce
  int reading = digitalRead(PIN_BUTTON); // Reads current button state
  if (reading != previous_button_state) { // If state changed
    last_debounce_time = millis(); // Resets debounce timer
  }
  if ((millis() - last_debounce_time) > debounce_ms) { // If state is stable
    if (reading != current_button_state) { // If stable state differs from recorded state
      current_button_state = reading; // Updates current state
      String button_payload = "";
      if (current_button_state == LOW) { // If button is pressed
        if (!is_button_alarm_active) {
          tone(PIN_BUZZER, BUZZER_FREQUENCY); // Starts buzzer
          button_payload = "{\"description\":\"The patient pressed the button.\", \"state_code\":1}"; // Creates JSON payload
          is_button_alarm_active = true; // Marks alarm as active
        }
      } else { // If button is released
        if (is_button_alarm_active) {
          noTone(PIN_BUZZER); // Stops buzzer
          button_payload = "{\"description\":\"Button is not pressed.\", \"state_code\":0}"; // Creates JSON payload
          is_button_alarm_active = false; // Marks alarm as inactive
        }
      }
      if (button_payload != "" && mqtt_client.connected()) {
        mqtt_client.publish(TOPIC_BUTTON, button_payload.c_str()); // Publishes button state
      } else if (button_payload != "" && !mqtt_client.connected()) {
         Serial.println("MQTT is not connected. Cannot send button state.");
      }
    }
  }
  previous_button_state = reading; // Saves current state for next iteration

  // Interval for BPM/SpO2 sending and MAX30102 collection start
  unsigned long current_ms = millis();
  if (current_ms - last_send_time >= other_sensor_interval) { // Checks whether time interval has passed
    last_send_time = current_ms; // Resets timer
    send_non_ECG_MQTT(); // Sends BPM/SpO2/electrode status data

    if (!is_max_collection_in_progress) { // If data is not already being collected
        long ir_value = pulse_sensor.getIR(); // Reads IR value to detect finger
        if (ir_value >= FINGER_IR_THRESHOLD) { // If finger is present
            collected_sample_count = 0; // Resets sample counter
            is_max_collection_in_progress = true; // Starts collection
        } else { // If finger is not present
            if (current_bpm != 0.0 || current_spo2 != 0.0) { // If there were previously valid values
                current_bpm = 0.0; // Resets current BPM value
                current_spo2 = 0.0; // Resets current SpO2 value
                is_bpm_history_full = false; // Resets BPM average history
                bpm_history_index = 0;
                for (int i = 0; i < BPM_AVERAGE_SAMPLE_COUNT; ++i) bpm_history[i] = 0.0;
                bpm_average_for_send = 0.0;
            }
        }
    }
  }
}
