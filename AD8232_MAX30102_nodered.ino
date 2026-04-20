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
const float TENSIUNE_REFERINTA_ADC = 0.0033; // ADC reference voltage in Volts (3.3V); using 0.0033 for correct Grafana representation
const float VALOARE_MAX_ADC = 4095.0; // Maximum value for 12-bit ADC (2^12 - 1)

// ECG filter parameters
const float FRECVENTA_ESANTIONARE = 200.0; // Frequency used to read the ECG signal (Hz)
const float TIMP_ESANTIONARE = 1.0 / FRECVENTA_ESANTIONARE; // Sampling period, calculated from frequency
const float FRECVENTA_TAIERE_FTJ = 25.0; // Cutoff frequency for the low-pass filter (removes high-frequency noise)
const float RC_FTJ = 1.0 / (2.0 * PI * FRECVENTA_TAIERE_FTJ); // Time constant for LPF
const float ALFA_FTJ = TIMP_ESANTIONARE / (RC_FTJ + TIMP_ESANTIONARE); // Smoothing factor for LPF
volatile float iesire_ftj_anterioara = 0.0; // Stores previous LPF output value

const float FRECVENTA_TAIERE_FTS = 0.5; // Cutoff frequency for the high-pass filter (removes DC component)
const float RC_FTS = 1.0 / (2.0 * PI * FRECVENTA_TAIERE_FTS); // Time constant for HPF
const float ALFA_FTS = RC_FTS / (RC_FTS + TIMP_ESANTIONARE); // Smoothing factor for HPF
volatile float iesire_fts_anterioara = 0.0; // Stores previous HPF output value
volatile float intrare_fts_anterioara = 0.0; // Stores previous HPF input value

volatile float ecg_filtrat_curent_mv = 0.0; // Current filtered ECG signal value, in millivolts
volatile bool ecg_pregatit_pentru_trimitere = false; // Flag indicating whether a new ECG value is ready to send

#define NR_MEDII_ECG 8 // Number of ECG samples used for moving average
float istoric_ecg[NR_MEDII_ECG]; // Array to store latest ECG values
int index_istoric_ecg = 0; // Current index in ECG history array
bool istoric_ecg_plin = false; // Flag indicating whether history array is full
float ecg_medie_pentru_trimitere = 0.0; // Average ECG value to be sent via MQTT

#define NR_MEDII_BPM 5 // Number of BPM samples used for moving average
float istoric_bpm[NR_MEDII_BPM]; // Array to store latest BPM values
int index_istoric_bpm = 0; // Current index in BPM history array
bool istoric_bpm_plin = false; // Flag indicating whether history array is full
float bpm_medie_pentru_trimitere = 0.0; // Average BPM value to be sent

#define PRAG_IR_DEGET 50000 // Minimum IR signal value to consider that a finger is on the sensor
#define MIN_BPM 30.0 // Minimum acceptable BPM value
#define MAX_BPM 220.0 // Maximum acceptable BPM value
#define MIN_SPO2 70.0 // Minimum acceptable SpO2 value
#define MAX_SPO2 100.0 // Maximum acceptable SpO2 value

#define PIN_SDA_MAX30102 21 // I2C SDA pin for MAX30102 sensor
#define PIN_SCL_MAX30102 22 // I2C SCL pin for MAX30102 sensor

#define PIN_ECG 36 // ADC pin for reading ECG signal
#define PIN_LO_PLUS 32 // Pin for detecting positive electrode contact, LA (Leads-Off+)
#define PIN_LO_MINUS 33 // Pin for detecting negative electrode contact, RA (Leads-Off-)

#define PIN_BUTON 14 // Pin for panic button
#define PIN_BUZZER 26 // Pin for alarm buzzer
#define FRECV_BUZZER 2000 // Frequency of buzzer sound (Hz)

const char* SSID_WIFI = "Device"; // WiFi network name
const char* PAROLA_WIFI = "46503190"; // WiFi network password
const char* SERVER_MQTT = "192.168.220.144"; // IP address of MQTT broker (Raspberry Pi)
const int PORT_MQTT = 1883; // Standard MQTT port
const char* UTILIZATOR_MQTT = "pi"; // Username for MQTT authentication
const char* PAROLA_MQTT = "Mariuspi"; // Password for MQTT authentication

// MQTT TOPICS for Node-RED
#define TOPIC_BPM "sensorData/bpm"
#define TOPIC_SPO2 "sensorData/spo2"
#define TOPIC_ECG "sensorData/ecg"
#define TOPIC_ELECTROZI "sensorData/lead_off"
#define TOPIC_BUTON "sensorData/button"

// Buffer for raw MAX30102 data
#define NR_ESANTIOANE_MAX30102 50 // Number of samples collected from MAX30102 before processing
uint32_t buffer_ir[NR_ESANTIOANE_MAX30102]; // Buffer for infrared (IR) values
uint32_t buffer_rosu[NR_ESANTIOANE_MAX30102]; // Buffer for red LED values
int32_t spo2; // Variable to store calculated SpO2 value
int8_t spo2_valid; // Flag indicating whether SpO2 value is valid
int32_t puls; // Variable to store calculated pulse value
int8_t puls_valid; // Flag indicating whether pulse value is valid

// Global state for non-blocking MAX30102 collection
volatile bool colectare_max_in_curs = false; // Flag indicating whether data is being collected from MAX30102
volatile int nr_esantioane_colectate = 0; // Counter for number of collected samples

MAX30105 senzor_puls; // Object for pulse sensor (MAX30105 class from library)
float bpm_curent = 0.0; // Current BPM value, before averaging
float spo2_curent = 0.0; // Current SpO2 value
volatile bool electrozi_lo_plus = false; // Current state of positive electrode
volatile bool electrozi_lo_minus = false; // Current state of negative electrode

WiFiClient client_wifi; // WiFi client object
PubSubClient client_mqtt(client_wifi); // MQTT client object using WiFi client

unsigned long ultimul_timp_trimitere = 0; // Stores timestamp of last sensor data send (BPM/SpO2)
const unsigned long interval_alti_senzori = 5000; // Interval in milliseconds for sending data (5 seconds)

unsigned long ultimul_timp_debounce = 0; // Stores timestamp of last button state change
const unsigned long debounce_ms = 50; // Debounce time for button (avoids multiple reads)
int stare_buton_anterioara = HIGH; // Previous button state (HIGH = not pressed)
int stare_buton_curenta = HIGH; // Current button state
bool buton_activ = false; // Flag indicating whether button alarm is active

esp_timer_handle_t timer_ecg; // Handle (identifier) for ECG high-precision timer

// Function declarations (prototypes)
void conectare_MQTT();
void procesare_max30102();
void trimite_ECG_MQTT();
void trimite_non_ECG_MQTT();
void callback_timer_ecg(void* arg);

// Constants for ECG moving average
const int numarValoriMedieECG = 20;
float valoriECG[numarValoriMedieECG];

void conectare_MQTT() {
  Serial.print("Attempting MQTT connection...");
  String id_client = "ESP32Client-"; // Creates a unique client ID
  id_client += String(random(0xffff), HEX);
  int incercari = 0;
  while (!client_mqtt.connected() && incercari < 5) { // Attempts to connect 5 times
    Serial.print("\nAttempt ");
    Serial.print(incercari + 1);
    Serial.print(" to MQTT broker: ");
    Serial.println(SERVER_MQTT);
    if (client_mqtt.connect(id_client.c_str(), UTILIZATOR_MQTT, PAROLA_MQTT)) { // Connect with ID, username, and password
      Serial.println("Connected to MQTT");
    } else {
      Serial.print("MQTT connection failed, rc=");
      Serial.print(client_mqtt.state()); // Displays error code
      Serial.println(" retry in 5 seconds");
      delay(5000); // Waits 5 seconds before retrying
    }
    incercari++;
  }
  if (!client_mqtt.connected()) {
    Serial.println("Could not connect to MQTT broker after multiple attempts.");
  }
}

// Non-blocking MAX30102 processing
void procesare_max30102() {
    if (!colectare_max_in_curs) { // Runs only if a collection is in progress
        return;
    }
    if (nr_esantioane_colectate < NR_ESANTIOANE_MAX30102) { // Checks whether buffer is not full
        if (senzor_puls.available()) { // Checks whether new data exists in sensor FIFO
            buffer_rosu[nr_esantioane_colectate] = senzor_puls.getFIFORed(); // Reads Red value
            buffer_ir[nr_esantioane_colectate] = senzor_puls.getFIFOIR(); // Reads IR value
            senzor_puls.nextSample(); // Moves to next FIFO sample
            nr_esantioane_colectate++; // Increments sample counter

            if (nr_esantioane_colectate >= NR_ESANTIOANE_MAX30102) { // If buffer is full
                // Calculates SpO2 and pulse using buffer data
                maxim_heart_rate_and_oxygen_saturation(buffer_ir, NR_ESANTIOANE_MAX30102, buffer_rosu, &spo2, &spo2_valid, &puls, &puls_valid);

                if (spo2_valid && spo2 >= MIN_SPO2 && spo2 <= MAX_SPO2) { // Validates SpO2
                    spo2_curent = (float)spo2;
                } else {
                    spo2_curent = 0.0; // Sets to 0 if invalid
                }

                if (puls_valid && puls >= MIN_BPM && puls <= MAX_BPM) { // Validates pulse
                    bpm_curent = (float)puls;
                } else {
                    bpm_curent = 0.0; // Sets to 0 if invalid
                }
                colectare_max_in_curs = false; // Stops collection flag
            }
        }
    }
}

void trimite_ECG_MQTT() {
  if (!client_mqtt.connected()) { // Checks MQTT connection
    return;
  }
  float esantion_ecg = ecg_filtrat_curent_mv; // Gets latest filtered ECG value
  istoric_ecg[index_istoric_ecg] = esantion_ecg; // Adds value to average history
  index_istoric_ecg = (index_istoric_ecg + 1) % NR_MEDII_ECG; // Updates circular index
  if (!istoric_ecg_plin && index_istoric_ecg == 0) {
    istoric_ecg_plin = true; // Marks history as full
  }

  if (istoric_ecg_plin) { // Calculates average if history is full
    float suma = 0;
    for (int i = 0; i < NR_MEDII_ECG; i++) {
      suma += istoric_ecg[i];
    }
    ecg_medie_pentru_trimitere = suma / NR_MEDII_ECG;
  } else { // Otherwise, use current value
    ecg_medie_pentru_trimitere = esantion_ecg;
  }

  char payload_ecg[20]; // Buffer for MQTT payload
  dtostrf(ecg_medie_pentru_trimitere, 8, 5, payload_ecg); // Converts float value to string with 5 decimals
  payload_ecg[19] = '\0'; // Ensures proper string termination
  if (!client_mqtt.publish(TOPIC_ECG, payload_ecg)) { // Publishes on ECG topic
    Serial.println("ECG MQTT send failed");
  }
}

void trimite_non_ECG_MQTT() {
  if (!client_mqtt.connected()) { // Checks MQTT connection
    return;
  }
  char payload_numeric[20]; // Buffer for numeric payloads

  // Update and calculate BPM moving average
  istoric_bpm[index_istoric_bpm] = bpm_curent; // Adds current BPM value to history
  index_istoric_bpm = (index_istoric_bpm + 1) % NR_MEDII_BPM; // Updates circular index
  if (!istoric_bpm_plin && index_istoric_bpm == 0) {
    istoric_bpm_plin = true; // Marks history as full
  }

  float suma_bpm_calc = 0.0;
  int count_samples_for_avg = 0;

  if (istoric_bpm_plin) { // Calculates average if history is full
    for (int i = 0; i < NR_MEDII_BPM; i++) {
      suma_bpm_calc += istoric_bpm[i];
    }
    count_samples_for_avg = NR_MEDII_BPM;
  } else { // Otherwise, calculates average on available samples
    for (int i = 0; i < index_istoric_bpm; i++) {
      suma_bpm_calc += istoric_bpm[i];
    }
    count_samples_for_avg = index_istoric_bpm;
  }

  if (count_samples_for_avg > 0) {
    bpm_medie_pentru_trimitere = suma_bpm_calc / count_samples_for_avg;
  } else {
    bpm_medie_pentru_trimitere = bpm_curent; // Uses current value if no samples are available for average
  }

  // BPM
  sprintf(payload_numeric, "%.2f", bpm_medie_pentru_trimitere); // Formats BPM value with 2 decimals
  client_mqtt.publish(TOPIC_BPM, payload_numeric); // Publishes on BPM topic

  // SpO2
  sprintf(payload_numeric, "%.2f", spo2_curent); // Formats SpO2 value with 2 decimals
  client_mqtt.publish(TOPIC_SPO2, payload_numeric); // Publishes on SpO2 topic

  // Electrode status (Lead status) in JSON format
  String lead_status_description = "";
  int lead_status_code = 0;
  if (electrozi_lo_plus && electrozi_lo_minus) {
    lead_status_description = "Both electrodes are disconnected.";
    lead_status_code = 3;
  } else if (electrozi_lo_plus) {
    lead_status_description = "LA electrode is disconnected.";
    lead_status_code = 1;
  } else if (electrozi_lo_minus) {
    lead_status_description = "RA electrode is disconnected.";
    lead_status_code = 2;
  } else {
    lead_status_description = "Both electrodes are connected.";
    lead_status_code = 0;
  }
  String lead_status_payload_json = "{\"description\":\"" + lead_status_description + "\", \"code\":" + String(lead_status_code) + "}"; // Creates JSON payload
  client_mqtt.publish(TOPIC_ELECTROZI, lead_status_payload_json.c_str()); // Publishes on electrode topic
}

// Timer callback function, runs at each ECG sample
void callback_timer_ecg(void* arg) {
  int valoare_adc = analogRead(PIN_ECG); // Reads raw value from ADC
  float tensiune = (valoare_adc / VALOARE_MAX_ADC) * TENSIUNE_REFERINTA_ADC; // Converts to voltage
  float iesire_ftj = ALFA_FTJ * tensiune + (1 - ALFA_FTJ) * iesire_ftj_anterioara; // Applies low-pass filter
  iesire_ftj_anterioara = iesire_ftj; // Saves filter state
  float iesire_fts = ALFA_FTS * (iesire_fts_anterioara + iesire_ftj - intrare_fts_anterioara); // Applies high-pass filter
  intrare_fts_anterioara = iesire_ftj; // Saves filter state
  iesire_fts_anterioara = iesire_fts; // Saves filter state
  ecg_filtrat_curent_mv = iesire_fts * 1000.0; // Converts filtered voltage to millivolts
  ecg_pregatit_pentru_trimitere = true; // Sets flag for sending
  electrozi_lo_plus = digitalRead(PIN_LO_PLUS); // Reads + electrode state
  electrozi_lo_minus = digitalRead(PIN_LO_MINUS); // Reads - electrode state
}

void setup() {
  Serial.begin(115200); // Initializes serial communication
  Wire.begin(PIN_SDA_MAX30102, PIN_SCL_MAX30102); // Initializes I2C communication on specified pins

  if (!senzor_puls.begin(Wire, I2C_SPEED_FAST)) { // Initializes MAX30102 sensor
    Serial.println("MAX30102 was not found. Check connections/power supply.");
    while (1); // Stops execution if sensor is not found
  }
  Serial.println("MAX30102 initialized.");

  // MAX30102 configuration: LED power, sample average, mode (Red+IR), sample rate, pulse width, ADC range
  senzor_puls.setup(0x1F, 4, 2, 100, 411, 4096);

  randomSeed(micros()); // Initializes random number generator
  pinMode(PIN_BUTON, INPUT_PULLUP); // Configures button pin as input with pull-up resistor
  pinMode(PIN_BUZZER, OUTPUT); // Configures buzzer pin as output
  noTone(PIN_BUZZER); // Stops buzzer at startup

  for(int i=0; i<NR_MEDII_ECG; ++i) istoric_ecg[i] = 0.0; // Initializes ECG history with 0
  for(int i=0; i<NR_MEDII_BPM; ++i) istoric_bpm[i] = 0.0; // Initializes BPM history with 0

  Serial.print("Connecting to WiFi: ");
  Serial.println(SSID_WIFI);
  WiFi.begin(SSID_WIFI, PAROLA_WIFI); // Starts WiFi connection
  int incercari_wifi = 0;
  while (WiFi.status() != WL_CONNECTED && incercari_wifi < 20) { // Waits for connection
    delay(500);
    Serial.print(".");
    incercari_wifi++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP()); // Displays obtained IP address
  } else {
    Serial.println("\nWiFi connection failed. Stopping.");
    while(true) { delay(1000); } // Stops execution if WiFi connection cannot be established
  }

  client_mqtt.setServer(SERVER_MQTT, PORT_MQTT); // Sets MQTT server
  conectare_MQTT(); // Connects to MQTT broker

  pinMode(PIN_ECG, INPUT); // Configures ECG pin as input
  pinMode(PIN_LO_PLUS, INPUT); // Configures LO+ pin as input
  pinMode(PIN_LO_MINUS, INPUT); // Configures LO- pin as input
  analogSetPinAttenuation(PIN_ECG, ADC_11db); // Sets attenuation to allow reading voltages up to 3.3V

  // Initialize filters with a first reading to avoid startup jump
  int valoare_initiala = analogRead(PIN_ECG);
  float tensiune_initiala = (valoare_initiala / VALOARE_MAX_ADC) * TENSIUNE_REFERINTA_ADC;
  iesire_ftj_anterioara = tensiune_initiala;
  intrare_fts_anterioara = tensiune_initiala;
  iesire_fts_anterioara = 0.0;

  // Configure and start timer for ECG sampling
  const esp_timer_create_args_t args_timer_ecg = {
      .callback = &callback_timer_ecg, // Function to be called by timer
      .name = "timer_ecg" // Timer name
  };
  esp_timer_create(&args_timer_ecg, &timer_ecg); // Creates timer
  esp_timer_start_periodic(timer_ecg, (uint64_t)(TIMP_ESANTIONARE * 1000000)); // Starts periodic timer
}

void loop() {
  senzor_puls.check(); // Checks MAX30102 sensor state (e.g., reads FIFO data)
  procesare_max30102(); // Processes collected MAX30102 data, if needed

  if (WiFi.status() != WL_CONNECTED) { // Checks whether WiFi connection is still active
    Serial.println("WiFi disconnected. Reconnecting...");
    WiFi.begin(SSID_WIFI, PAROLA_WIFI); // Attempts reconnection
    int incercari_wifi = 0;
    while (WiFi.status() != WL_CONNECTED && incercari_wifi < 20) {
      delay(500);
      Serial.print(".");
      incercari_wifi++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi reconectat.");
      conectare_MQTT(); // Reconnect to MQTT after WiFi returns
    } else {
      Serial.println("\nWiFi reconnection failed.");
      delay(5000);
      return; // Exits loop to retry later
    }
  }

  if (!client_mqtt.connected()) { // Checks whether MQTT client is still connected
    conectare_MQTT(); // Reconnects if needed
  }
  client_mqtt.loop(); // Keeps MQTT connection alive and processes incoming messages

  if (ecg_pregatit_pentru_trimitere) { // If a new ECG value is available
    trimite_ECG_MQTT(); // Sends value via MQTT
    ecg_pregatit_pentru_trimitere = false; // Resets flag
  }

  // Logic for panic button with debounce
  int citire = digitalRead(PIN_BUTON); // Reads current button state
  if (citire != stare_buton_anterioara) { // If state changed
    ultimul_timp_debounce = millis(); // Resets debounce timer
  }
  if ((millis() - ultimul_timp_debounce) > debounce_ms) { // If state is stable
    if (citire != stare_buton_curenta) { // If stable state differs from recorded state
      stare_buton_curenta = citire; // Updates current state
      String payload_buton = "";
      if (stare_buton_curenta == LOW) { // If button is pressed
        if (!buton_activ) {
          tone(PIN_BUZZER, FRECV_BUZZER); // Starts buzzer
          payload_buton = "{\"description\":\"The patient pressed the button.\", \"state_code\":1}"; // Creates JSON payload
          buton_activ = true; // Marks alarm as active
        }
      } else { // If button is released
        if (buton_activ) {
          noTone(PIN_BUZZER); // Stops buzzer
          payload_buton = "{\"description\":\"Button is not pressed.\", \"state_code\":0}"; // Creates JSON payload
          buton_activ = false; // Marks alarm as inactive
        }
      }
      if (payload_buton != "" && client_mqtt.connected()) {
        client_mqtt.publish(TOPIC_BUTON, payload_buton.c_str()); // Publishes button state
      } else if (payload_buton != "" && !client_mqtt.connected()) {
         Serial.println("MQTT is not connected. Cannot send button state.");
      }
    }
  }
  stare_buton_anterioara = citire; // Saves current state for next iteration

  // Interval for BPM/SpO2 sending and MAX30102 collection start
  unsigned long ms_curent = millis();
  if (ms_curent - ultimul_timp_trimitere >= interval_alti_senzori) { // Checks whether time interval has passed
    ultimul_timp_trimitere = ms_curent; // Resets timer
    trimite_non_ECG_MQTT(); // Sends BPM/SpO2/electrode status data

    if (!colectare_max_in_curs) { // If data is not already being collected
        long valoare_ir = senzor_puls.getIR(); // Reads IR value to detect finger
        if (valoare_ir >= PRAG_IR_DEGET) { // If finger is present
            nr_esantioane_colectate = 0; // Resets sample counter
            colectare_max_in_curs = true; // Starts collection
        } else { // If finger is not present
            if (bpm_curent != 0.0 || spo2_curent != 0.0) { // If there were previously valid values
                bpm_curent = 0.0; // Resets current BPM value
                spo2_curent = 0.0; // Resets current SpO2 value
                istoric_bpm_plin = false; // Resets BPM average history
                index_istoric_bpm = 0;
                for (int i = 0; i < NR_MEDII_BPM; ++i) istoric_bpm[i] = 0.0;
                bpm_medie_pentru_trimitere = 0.0;
            }
        }
    }
  }
}
