# ECG
This project implements a real-time ECG monitoring system using an ESP32 microcontroller and an AD8232 ECG sensor. The ESP32 reads analog ECG signals via its ADC, processes the data, and publishes it over MQTT using a Wi-Fi connection. In addition to ECG waveform data, the ESP32 can also transmit heart rate (BPM), SpO₂ (via MAX30102 if used), and lead-off detection information on separate MQTT topics.

The backend of the system is powered by Node-RED, which subscribes to these MQTT topics using a JSON-based flow. This flow:

Parses and filters the incoming health data,

Visualizes the ECG waveform in real-time using chart nodes on a Node-RED dashboard,

Displays numeric data like BPM, SpO₂, and status indicators,

Forwards all the data to InfluxDB, a time-series database, where it is stored for historical analysis.

To visualize the stored data, the project integrates Grafana, which queries InfluxDB and presents real-time and historical trends of ECG and other metrics. Grafana is also configured with alerting rules — for example, triggering notifications if BPM exceeds a critical threshold or if ECG signal loss is detected.

Alerts can be sent to multiple contact points such as:

Email (SMTP/Gmail),

Telegram (via bot),

Microsoft Teams or other services using webhooks.
