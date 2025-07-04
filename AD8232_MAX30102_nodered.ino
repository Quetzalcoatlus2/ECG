#include <Wire.h> // Biblioteca pentru comunicație I2C (necesară pentru senzorul MAX30102)
#include <WiFi.h> // Biblioteca pentru gestionarea conexiunii WiFi
#define MQTT_MAX_PACKET_SIZE 512 // Mărește dimensiunea maximă a pachetului MQTT pentru a permite payload-uri mai mari
#include <PubSubClient.h> // Biblioteca pentru clientul MQTT
#include "MAX30105.h" // Biblioteca SparkFun pentru senzorul de puls și SpO2 (compatibilă și cu MAX30102)
#include "heartRate.h" // Header pentru algoritmul de calcul al ritmului cardiac
#include "spo2_algorithm.h" // Header pentru algoritmul de calcul al SpO2
#include "esp_timer.h" // Biblioteca pentru timere de înaltă precizie pe ESP32
#include <math.h> // Biblioteca matematică, necesară pentru constanta PI

// Tensiune de referință ADC
const float TENSIUNE_REFERINTA_ADC = 0.0033; // Tensiunea de referință a ADC-ului în Volți (3.3V), folosim 0.0033 pentru reprezentare corectă în Grafana
const float VALOARE_MAX_ADC = 4095.0; // Valoarea maximă pentru ADC-ul de 12 biți (2^12 - 1)

// Parametri filtru pentru ECG
const float FRECVENTA_ESANTIONARE = 200.0; // Frecvența cu care citim semnalul ECG (Hz)
const float TIMP_ESANTIONARE = 1.0 / FRECVENTA_ESANTIONARE; // Perioada de eșantionare, calculată din frecvență
const float FRECVENTA_TAIERE_FTJ = 25.0; // Frecvența de tăiere pentru Filtrul Trece-Jos (elimină zgomotul de înaltă frecvență)
const float RC_FTJ = 1.0 / (2.0 * PI * FRECVENTA_TAIERE_FTJ); // Constanta de timp pentru FTJ
const float ALFA_FTJ = TIMP_ESANTIONARE / (RC_FTJ + TIMP_ESANTIONARE); // Factorul de netezire pentru FTJ
volatile float iesire_ftj_anterioara = 0.0; // Stochează valoarea anterioară a filtrului FTJ

const float FRECVENTA_TAIERE_FTS = 0.5; // Frecvența de tăiere pentru Filtrul Trece-Sus (elimină componenta DC)
const float RC_FTS = 1.0 / (2.0 * PI * FRECVENTA_TAIERE_FTS); // Constanta de timp pentru FTS
const float ALFA_FTS = RC_FTS / (RC_FTS + TIMP_ESANTIONARE); // Factorul de netezire pentru FTS
volatile float iesire_fts_anterioara = 0.0; // Stochează valoarea anterioară a ieșirii FTS
volatile float intrare_fts_anterioara = 0.0; // Stochează valoarea anterioară a intrării FTS

volatile float ecg_filtrat_curent_mv = 0.0; // Valoarea curentă a semnalului ECG filtrat, în milivolți
volatile bool ecg_pregatit_pentru_trimitere = false; // Flag care indică dacă o nouă valoare ECG este gata de trimis

#define NR_MEDII_ECG 8 // Numărul de eșantioane ECG folosite pentru media mobilă
float istoric_ecg[NR_MEDII_ECG]; // Array pentru a stoca ultimele valori ECG
int index_istoric_ecg = 0; // Indexul curent în array-ul de istoric ECG
bool istoric_ecg_plin = false; // Flag care indică dacă array-ul de istoric este plin
float ecg_medie_pentru_trimitere = 0.0; // Valoarea medie a ECG care va fi trimisă prin MQTT

#define NR_MEDII_BPM 5 // Numărul de eșantioane BPM folosite pentru media mobilă
float istoric_bpm[NR_MEDII_BPM]; // Array pentru a stoca ultimele valori BPM
int index_istoric_bpm = 0; // Indexul curent în array-ul de istoric BPM
bool istoric_bpm_plin = false; // Flag care indică dacă array-ul de istoric este plin
float bpm_medie_pentru_trimitere = 0.0; // Valoarea medie a BPM care va fi trimisă

#define PRAG_IR_DEGET 50000 // Valoarea minimă a semnalului IR pentru a considera că degetul este pe senzor
#define MIN_BPM 30.0 // Valoarea minimă acceptabilă pentru BPM
#define MAX_BPM 220.0 // Valoarea maximă acceptabilă pentru BPM
#define MIN_SPO2 70.0 // Valoarea minimă acceptabilă pentru SpO2
#define MAX_SPO2 100.0 // Valoarea maximă acceptabilă pentru SpO2

#define PIN_SDA_MAX30102 21 // Pinul I2C SDA pentru senzorul MAX30102
#define PIN_SCL_MAX30102 22 // Pinul I2C SCL pentru senzorul MAX30102

#define PIN_ECG 36 // Pinul ADC pentru citirea semnalului ECG
#define PIN_LO_PLUS 32 // Pinul pentru detectarea contactului electrodului pozitiv, LA (Leads-Off+)
#define PIN_LO_MINUS 33 // Pinul pentru detectarea contactului electrodului negativ, RA (Leads-Off-)

#define PIN_BUTON 14 // Pinul pentru butonul de panică
#define PIN_BUZZER 26 // Pinul pentru buzzer-ul de alarmă
#define FRECV_BUZZER 2000 // Frecvența sunetului produs de buzzer (în Hz)

const char* SSID_WIFI = "Device"; // Numele rețelei WiFi
const char* PAROLA_WIFI = "46503190"; // Parola rețelei WiFi
const char* SERVER_MQTT = "192.168.220.144"; // Adresa IP a broker-ului MQTT (Raspberry Pi)
const int PORT_MQTT = 1883; // Portul standard pentru MQTT
const char* UTILIZATOR_MQTT = "pi"; // Utilizatorul pentru autentificare la MQTT
const char* PAROLA_MQTT = "Mariuspi"; // Parola pentru autentificare la MQTT

// TOPICURI MQTT pentru Node-RED
#define TOPIC_BPM "sensorData/bpm"
#define TOPIC_SPO2 "sensorData/spo2"
#define TOPIC_ECG "sensorData/ecg"
#define TOPIC_ELECTROZI "sensorData/lead_off"
#define TOPIC_BUTON "sensorData/button"

// Buffer pentru date brute MAX30102
#define NR_ESANTIOANE_MAX30102 50 // Numărul de eșantioane colectate de la MAX30102 înainte de procesare
uint32_t buffer_ir[NR_ESANTIOANE_MAX30102]; // Buffer pentru valorile infraroșu (IR)
uint32_t buffer_rosu[NR_ESANTIOANE_MAX30102]; // Buffer pentru valorile LED-ului roșu
int32_t spo2; // Variabilă pentru a stoca valoarea calculată a SpO2
int8_t spo2_valid; // Flag care indică dacă valoarea SpO2 este validă
int32_t puls; // Variabilă pentru a stoca valoarea calculată a pulsului
int8_t puls_valid; // Flag care indică dacă valoarea pulsului este validă

// Stare globală pentru colectare non-blocantă MAX30102
volatile bool colectare_max_in_curs = false; // Flag care indică dacă se colectează date de la MAX30102
volatile int nr_esantioane_colectate = 0; // Contor pentru numărul de eșantioane colectate

MAX30105 senzor_puls; // Obiectul pentru senzorul de puls (clasa MAX30105 din bibliotecă)
float bpm_curent = 0.0; // Valoarea curentă a BPM, înainte de mediere
float spo2_curent = 0.0; // Valoarea curentă a SpO2
volatile bool electrozi_lo_plus = false; // Starea curentă a electrodului pozitiv
volatile bool electrozi_lo_minus = false; // Starea curentă a electrodului negativ

WiFiClient client_wifi; // Obiectul client WiFi
PubSubClient client_mqtt(client_wifi); // Obiectul client MQTT, care folosește clientul WiFi

unsigned long ultimul_timp_trimitere = 0; // Stochează timpul ultimei trimiteri a datelor de la senzori (BPM/SpO2)
const unsigned long interval_alti_senzori = 5000; // Intervalul în milisecunde pentru trimiterea datelor (5 secunde)

unsigned long ultimul_timp_debounce = 0; // Stochează timpul ultimei schimbări de stare a butonului
const unsigned long debounce_ms = 50; // Timpul de debounce pentru buton (evită citirile multiple)
int stare_buton_anterioara = HIGH; // Starea anterioară a butonului (HIGH = neapăsat)
int stare_buton_curenta = HIGH; // Starea curentă a butonului
bool buton_activ = false; // Flag care indică dacă alarma butonului este activă

esp_timer_handle_t timer_ecg; // Handle (identificator) pentru timer-ul de înaltă precizie al ECG

// Declarații de funcții (prototipuri)
void conectare_MQTT();
void procesare_max30102();
void trimite_ECG_MQTT();
void trimite_non_ECG_MQTT();
void callback_timer_ecg(void* arg);

// Constante pentru media mobilă ECG
const int numarValoriMedieECG = 20;
float valoriECG[numarValoriMedieECG];

void conectare_MQTT() {
  Serial.print("Se încearcă conectarea la MQTT...");
  String id_client = "ESP32Client-"; // Creează un ID de client unic
  id_client += String(random(0xffff), HEX);
  int incercari = 0;
  while (!client_mqtt.connected() && incercari < 5) { // Încearcă să se conecteze de 5 ori
    Serial.print("\nÎncercare ");
    Serial.print(incercari + 1);
    Serial.print(" la broker MQTT: ");
    Serial.println(SERVER_MQTT);
    if (client_mqtt.connect(id_client.c_str(), UTILIZATOR_MQTT, PAROLA_MQTT)) { // Conectare cu ID, utilizator și parolă
      Serial.println("Conectat la MQTT");
    } else {
      Serial.print("Conectare MQTT eșuată, rc=");
      Serial.print(client_mqtt.state()); // Afișează codul de eroare
      Serial.println(" reîncercare în 5 secunde");
      delay(5000); // Așteaptă 5 secunde înainte de a reîncerca
    }
    incercari++;
  }
  if (!client_mqtt.connected()) {
    Serial.println("Nu s-a putut conecta la brokerul MQTT după mai multe încercări.");
  }
}

// Procesare non-blocantă MAX30102
void procesare_max30102() {
    if (!colectare_max_in_curs) { // Rulează doar dacă o colectare este în desfășurare
        return;
    }
    if (nr_esantioane_colectate < NR_ESANTIOANE_MAX30102) { // Verifică dacă bufferul nu este plin
        if (senzor_puls.available()) { // Verifică dacă există date noi în FIFO-ul senzorului
            buffer_rosu[nr_esantioane_colectate] = senzor_puls.getFIFORed(); // Citește valoarea Red
            buffer_ir[nr_esantioane_colectate] = senzor_puls.getFIFOIR(); // Citește valoarea IR
            senzor_puls.nextSample(); // Trece la următorul eșantion din FIFO
            nr_esantioane_colectate++; // Incrementează contorul de eșantioane

            if (nr_esantioane_colectate >= NR_ESANTIOANE_MAX30102) { // Dacă bufferul s-a umplut
                // Calculează SpO2 și pulsul folosind datele din buffere
                maxim_heart_rate_and_oxygen_saturation(buffer_ir, NR_ESANTIOANE_MAX30102, buffer_rosu, &spo2, &spo2_valid, &puls, &puls_valid);

                if (spo2_valid && spo2 >= MIN_SPO2 && spo2 <= MAX_SPO2) { // Validează SpO2
                    spo2_curent = (float)spo2;
                } else {
                    spo2_curent = 0.0; // Setează 0 dacă este invalid
                }

                if (puls_valid && puls >= MIN_BPM && puls <= MAX_BPM) { // Validează pulsul
                    bpm_curent = (float)puls;
                } else {
                    bpm_curent = 0.0; // Setează 0 dacă este invalid
                }
                colectare_max_in_curs = false; // Oprește flagul de colectare
            }
        }
    }
}

void trimite_ECG_MQTT() {
  if (!client_mqtt.connected()) { // Verifică conexiunea MQTT
    return;
  }
  float esantion_ecg = ecg_filtrat_curent_mv; // Preia ultima valoare ECG filtrată
  istoric_ecg[index_istoric_ecg] = esantion_ecg; // Adaugă valoarea în istoricul pentru medie
  index_istoric_ecg = (index_istoric_ecg + 1) % NR_MEDII_ECG; // Actualizează indexul circular
  if (!istoric_ecg_plin && index_istoric_ecg == 0) {
    istoric_ecg_plin = true; // Marchează istoricul ca fiind plin
  }

  if (istoric_ecg_plin) { // Calculează media dacă istoricul este plin
    float suma = 0;
    for (int i = 0; i < NR_MEDII_ECG; i++) {
      suma += istoric_ecg[i];
    }
    ecg_medie_pentru_trimitere = suma / NR_MEDII_ECG;
  } else { // Altfel, folosește valoarea curentă
    ecg_medie_pentru_trimitere = esantion_ecg;
  }

  char payload_ecg[20]; // Buffer pentru payload-ul MQTT
  dtostrf(ecg_medie_pentru_trimitere, 8, 5, payload_ecg); // Convertește valoarea float în string cu 5 zecimale
  payload_ecg[19] = '\0'; // Asigură terminarea corectă a string-ului
  if (!client_mqtt.publish(TOPIC_ECG, payload_ecg)) { // Publică pe topicul ECG
    Serial.println("Trimitere ECG MQTT eșuată");
  }
}

void trimite_non_ECG_MQTT() {
  if (!client_mqtt.connected()) { // Verifică conexiunea MQTT
    return;
  }
  char payload_numeric[20]; // Buffer pentru payload-urile numerice

  // Actualizare și calcul medie mobilă BPM
  istoric_bpm[index_istoric_bpm] = bpm_curent; // Adaugă valoarea curentă a BPM în istoric
  index_istoric_bpm = (index_istoric_bpm + 1) % NR_MEDII_BPM; // Actualizează indexul circular
  if (!istoric_bpm_plin && index_istoric_bpm == 0) {
    istoric_bpm_plin = true; // Marchează istoricul ca fiind plin
  }

  float suma_bpm_calc = 0.0;
  int count_samples_for_avg = 0;

  if (istoric_bpm_plin) { // Calculează media dacă istoricul este plin
    for (int i = 0; i < NR_MEDII_BPM; i++) {
      suma_bpm_calc += istoric_bpm[i];
    }
    count_samples_for_avg = NR_MEDII_BPM;
  } else { // Altfel, calculează media pe eșantioanele disponibile
    for (int i = 0; i < index_istoric_bpm; i++) {
      suma_bpm_calc += istoric_bpm[i];
    }
    count_samples_for_avg = index_istoric_bpm;
  }

  if (count_samples_for_avg > 0) {
    bpm_medie_pentru_trimitere = suma_bpm_calc / count_samples_for_avg;
  } else {
    bpm_medie_pentru_trimitere = bpm_curent; // Folosește valoarea curentă dacă nu sunt eșantioane în medie
  }

  // BPM
  sprintf(payload_numeric, "%.2f", bpm_medie_pentru_trimitere); // Formatează valoarea BPM cu 2 zecimale
  client_mqtt.publish(TOPIC_BPM, payload_numeric); // Publică pe topicul BPM

  // SpO2
  sprintf(payload_numeric, "%.2f", spo2_curent); // Formatează valoarea SpO2 cu 2 zecimale
  client_mqtt.publish(TOPIC_SPO2, payload_numeric); // Publică pe topicul SpO2

  // Starea electrozilor (Lead status) în format JSON
  String lead_status_description = "";
  int lead_status_code = 0;
  if (electrozi_lo_plus && electrozi_lo_minus) {
    lead_status_description = "Ambii electrozi sunt deconectați.";
    lead_status_code = 3;
  } else if (electrozi_lo_plus) {
    lead_status_description = "Electrodul LA este deconectat.";
    lead_status_code = 1;
  } else if (electrozi_lo_minus) {
    lead_status_description = "Electrodul RA este deconectat.";
    lead_status_code = 2;
  } else {
    lead_status_description = "Ambii electrozi sunt conectați.";
    lead_status_code = 0;
  }
  String lead_status_payload_json = "{\"description\":\"" + lead_status_description + "\", \"code\":" + String(lead_status_code) + "}"; // Creează payload-ul JSON
  client_mqtt.publish(TOPIC_ELECTROZI, lead_status_payload_json.c_str()); // Publică pe topicul de electrozi
}

// Funcția de callback a timer-ului, rulează la fiecare eșantionare ECG
void callback_timer_ecg(void* arg) {
  int valoare_adc = analogRead(PIN_ECG); // Citește valoarea brută de la ADC
  float tensiune = (valoare_adc / VALOARE_MAX_ADC) * TENSIUNE_REFERINTA_ADC; // Convertește în tensiune
  float iesire_ftj = ALFA_FTJ * tensiune + (1 - ALFA_FTJ) * iesire_ftj_anterioara; // Aplică Filtrul Trece-Jos
  iesire_ftj_anterioara = iesire_ftj; // Salvează starea filtrului
  float iesire_fts = ALFA_FTS * (iesire_fts_anterioara + iesire_ftj - intrare_fts_anterioara); // Aplică Filtrul Trece-Sus
  intrare_fts_anterioara = iesire_ftj; // Salvează starea filtrului
  iesire_fts_anterioara = iesire_fts; // Salvează starea filtrului
  ecg_filtrat_curent_mv = iesire_fts * 1000.0; // Convertește tensiunea filtrată în milivolți
  ecg_pregatit_pentru_trimitere = true; // Setează flagul pentru trimitere
  electrozi_lo_plus = digitalRead(PIN_LO_PLUS); // Citește starea electrodului +
  electrozi_lo_minus = digitalRead(PIN_LO_MINUS); // Citește starea electrodului -
}

void setup() {
  Serial.begin(115200); // Inițializează comunicația serială
  Wire.begin(PIN_SDA_MAX30102, PIN_SCL_MAX30102); // Inițializează comunicația I2C pe pinii specificați

  if (!senzor_puls.begin(Wire, I2C_SPEED_FAST)) { // Inițializează senzorul MAX30102
    Serial.println("MAX30102 nu a fost găsit. Verifică conexiunile/alimentarea.");
    while (1); // Oprește execuția dacă senzorul nu este găsit
  }
  Serial.println("MAX30102 inițializat.");

  // Configurare pentru MAX30102: putere LED, medie eșantioane, mod (Red+IR), rată eșantionare, lățime puls, rang ADC
  senzor_puls.setup(0x1F, 4, 2, 100, 411, 4096);

  randomSeed(micros()); // Inițializează generatorul de numere aleatorii
  pinMode(PIN_BUTON, INPUT_PULLUP); // Configurează pinul butonului ca intrare cu rezistor de pull-up
  pinMode(PIN_BUZZER, OUTPUT); // Configurează pinul buzzer-ului ca ieșire
  noTone(PIN_BUZZER); // Oprește buzzer-ul la pornire

  for(int i=0; i<NR_MEDII_ECG; ++i) istoric_ecg[i] = 0.0; // Inițializează istoricul ECG cu 0
  for(int i=0; i<NR_MEDII_BPM; ++i) istoric_bpm[i] = 0.0; // Inițializează istoricul BPM cu 0

  Serial.print("Se conectează la WiFi: ");
  Serial.println(SSID_WIFI);
  WiFi.begin(SSID_WIFI, PAROLA_WIFI); // Pornește conexiunea WiFi
  int incercari_wifi = 0;
  while (WiFi.status() != WL_CONNECTED && incercari_wifi < 20) { // Așteaptă conectarea
    delay(500);
    Serial.print(".");
    incercari_wifi++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectat");
    Serial.print("Adresa IP: ");
    Serial.println(WiFi.localIP()); // Afișează adresa IP obținută
  } else {
    Serial.println("\nConectare WiFi eșuată. Oprire.");
    while(true) { delay(1000); } // Oprește execuția dacă nu se poate conecta la WiFi
  }

  client_mqtt.setServer(SERVER_MQTT, PORT_MQTT); // Setează serverul MQTT
  conectare_MQTT(); // Se conectează la brokerul MQTT

  pinMode(PIN_ECG, INPUT); // Configurează pinul ECG ca intrare
  pinMode(PIN_LO_PLUS, INPUT); // Configurează pinul LO+ ca intrare
  pinMode(PIN_LO_MINUS, INPUT); // Configurează pinul LO- ca intrare
  analogSetPinAttenuation(PIN_ECG, ADC_11db); // Setează atenuarea pentru a permite citirea de tensiuni până la 3.3V

  // Inițializarea filtrelor cu o primă citire pentru a evita un salt la pornire
  int valoare_initiala = analogRead(PIN_ECG);
  float tensiune_initiala = (valoare_initiala / VALOARE_MAX_ADC) * TENSIUNE_REFERINTA_ADC;
  iesire_ftj_anterioara = tensiune_initiala;
  intrare_fts_anterioara = tensiune_initiala;
  iesire_fts_anterioara = 0.0;

  // Configurare și pornire timer pentru eșantionare ECG
  const esp_timer_create_args_t args_timer_ecg = {
      .callback = &callback_timer_ecg, // Funcția care va fi apelată de timer
      .name = "timer_ecg" // Numele timer-ului
  };
  esp_timer_create(&args_timer_ecg, &timer_ecg); // Creează timer-ul
  esp_timer_start_periodic(timer_ecg, (uint64_t)(TIMP_ESANTIONARE * 1000000)); // Pornește timer-ul periodic
}

void loop() {
  senzor_puls.check(); // Verifică starea senzorului MAX30102 (ex: citește datele din FIFO)
  procesare_max30102(); // Procesează datele colectate de la MAX30102, dacă este cazul

  if (WiFi.status() != WL_CONNECTED) { // Verifică dacă conexiunea WiFi este încă activă
    Serial.println("WiFi deconectat. Se reconectează...");
    WiFi.begin(SSID_WIFI, PAROLA_WIFI); // Încearcă reconectarea
    int incercari_wifi = 0;
    while (WiFi.status() != WL_CONNECTED && incercari_wifi < 20) {
      delay(500);
      Serial.print(".");
      incercari_wifi++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi reconectat.");
      conectare_MQTT(); // Reconectează-te la MQTT după ce WiFi-ul revine
    } else {
      Serial.println("\nReconectare WiFi eșuată.");
      delay(5000);
      return; // Iese din loop pentru a reîncerca mai târziu
    }
  }

  if (!client_mqtt.connected()) { // Verifică dacă clientul MQTT este încă conectat
    conectare_MQTT(); // Reconectează-te dacă este necesar
  }
  client_mqtt.loop(); // Menține conexiunea MQTT activă și procesează mesajele primite

  if (ecg_pregatit_pentru_trimitere) { // Dacă o nouă valoare ECG este disponibilă
    trimite_ECG_MQTT(); // Trimite valoarea prin MQTT
    ecg_pregatit_pentru_trimitere = false; // Resetează flagul
  }

  // Logica pentru butonul de panică cu debounce
  int citire = digitalRead(PIN_BUTON); // Citește starea curentă a butonului
  if (citire != stare_buton_anterioara) { // Dacă starea s-a schimbat
    ultimul_timp_debounce = millis(); // Resetează timer-ul de debounce
  }
  if ((millis() - ultimul_timp_debounce) > debounce_ms) { // Dacă starea este stabilă
    if (citire != stare_buton_curenta) { // Dacă starea stabilă este diferită de cea înregistrată
      stare_buton_curenta = citire; // Actualizează starea curentă
      String payload_buton = "";
      if (stare_buton_curenta == LOW) { // Dacă butonul este apăsat
        if (!buton_activ) {
          tone(PIN_BUZZER, FRECV_BUZZER); // Pornește buzzer-ul
          payload_buton = "{\"description\":\"Pacientul a apăsat butonul.\", \"state_code\":1}"; // Creează payload-ul JSON
          buton_activ = true; // Marchează alarma ca activă
        }
      } else { // Dacă butonul este eliberat
        if (buton_activ) {
          noTone(PIN_BUZZER); // Oprește buzzer-ul
          payload_buton = "{\"description\":\"Butonul nu e apăsat.\", \"state_code\":0}"; // Creează payload-ul JSON
          buton_activ = false; // Marchează alarma ca inactivă
        }
      }
      if (payload_buton != "" && client_mqtt.connected()) {
        client_mqtt.publish(TOPIC_BUTON, payload_buton.c_str()); // Publică starea butonului
      } else if (payload_buton != "" && !client_mqtt.connected()) {
         Serial.println("MQTT nu este conectat. Nu se poate trimite starea butonului.");
      }
    }
  }
  stare_buton_anterioara = citire; // Salvează starea curentă pentru următoarea iterație

  // Interval pentru trimitere BPM/SpO2 și inițiere colectare MAX30102
  unsigned long ms_curent = millis();
  if (ms_curent - ultimul_timp_trimitere >= interval_alti_senzori) { // Verifică dacă a trecut intervalul de timp
    ultimul_timp_trimitere = ms_curent; // Resetează timer-ul
    trimite_non_ECG_MQTT(); // Trimite datele de la BPM/SpO2/stare electrozi

    if (!colectare_max_in_curs) { // Dacă nu se colectează deja date
        long valoare_ir = senzor_puls.getIR(); // Citește valoarea IR pentru a detecta degetul
        if (valoare_ir >= PRAG_IR_DEGET) { // Dacă degetul este prezent
            nr_esantioane_colectate = 0; // Resetează contorul de eșantioane
            colectare_max_in_curs = true; // Pornește colectarea
        } else { // Dacă degetul nu este prezent
            if (bpm_curent != 0.0 || spo2_curent != 0.0) { // Dacă existau valori valide anterior
                bpm_curent = 0.0; // Resetează valoarea curentă a BPM
                spo2_curent = 0.0; // Resetează valoarea curentă a SpO2
                istoric_bpm_plin = false; // Resetează istoricul mediei BPM
                index_istoric_bpm = 0;
                for (int i = 0; i < NR_MEDII_BPM; ++i) istoric_bpm[i] = 0.0;
                bpm_medie_pentru_trimitere = 0.0;
            }
        }
    }
  }
}
