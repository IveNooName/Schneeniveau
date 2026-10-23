/******************************************************************************************************
 *  Schneeniveau-Messstation
 *  Ein Ultraschall-Distanzsensor (A01NYUB / DFRobot SEN0313) misst die Schneehöhe, bis zu 4 DS18B20
 *  messen Temperaturen. Die Messwerte werden via MQTT versendet.
 *
 *  Ausgelegt für Dauerbetrieb (Monate ohne Eingriff):
 *   - Hardware-Watchdog: hängt das Programm, startet der ESP neu
 *   - WLAN und MQTT werden im Betrieb automatisch neu verbunden, nichts blockiert endlos
 *   - alle Zeitvergleiche mit millis() sind überlauf-sicher (Überlauf nach ~49 Tagen)
 *   - keine String-Objekte im Dauerbetrieb (keine Heap-Fragmentierung)
 *   - Notfall-Neustart, wenn zu lange nichts gesendet werden konnte oder der Heap knapp wird
 *   - vorsorglicher Neustart in regelmässigen Abständen
 ********************************************************************************************************/

#include <Arduino.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <HardwareSerial.h>
#include <PubSubClient.h>
#include <esp_task_wdt.h>
#include "config.h"
#include "time.h"


// ============================================================================
// MARK: - Einstellungen
// ============================================================================

// Intervalle in Millisekunden
const uint32_t INTERVAL_DISTANCE_MS    = 10UL * 1000;
const uint32_t INTERVAL_TEMPERATURE_MS = 30UL * 1000;
const uint32_t INTERVAL_HEARTBEAT_MS   = 60UL * 1000;
const uint32_t INTERVAL_MQTT_RETRY_MS  = 10UL * 1000;
const uint32_t INTERVAL_SENSOR_SCAN_MS = 10UL * 60 * 1000;   // DS18B20 neu suchen, falls keine gefunden

// Sicherheitsgrenzen
const uint32_t WDT_TIMEOUT_S           = 60;                    // Watchdog
const uint32_t WIFI_LOST_RESTART_MS    = 10UL * 60 * 1000;      // so lange ohne WLAN -> Neustart
const uint32_t NO_PUBLISH_RESTART_MS   = 30UL * 60 * 1000;      // so lange nichts gesendet -> Neustart
const uint32_t PREVENTIVE_RESTART_MS   = 7UL * 24 * 60 * 60 * 1000;  // vorsorglicher Neustart alle 7 Tage
const uint32_t MIN_FREE_HEAP           = 20000;                 // Bytes
const uint32_t DISPLAY_ON_MS           = 10UL * 60 * 1000;      // OLED nach Start so lange an (Einbrennschutz)

// MQTT
const uint16_t    MQTT_PORT        = 1883;
const char*       MQTT_CLIENT_ID   = "ESP32_Gruben";
const char*       TOPIC_STATUS     = "gruben/pub/status";
const char*       TOPIC_DISTANCE   = "gruben/pub/schneehoehe";
const char*       TOPIC_TEMP_FMT   = "gruben/pub/temperatur/%d";
const char*       TOPIC_UPTIME     = "gruben/pub/uptime";
const char*       TOPIC_RSSI       = "gruben/pub/rssi";
const char*       TOPIC_SUBSCRIBE  = "gruben/sub/#";
const char*       MSG_ONLINE       = "online";
const char*       MSG_OFFLINE      = "offline";

// DS18B20
#define ONE_WIRE_BUS 22
#define TEMPERATURE_PRECISION 12
#define MAX_DS1820 4

// Ultraschallsensor A01NYUB
#define A01NYUB_VCC_PIN   17
#define A01NYUB_RX_PIN    12
#define A01NYUB_TX_PIN    13
#define A01NYUB_BAUD      9600
const uint32_t A01NYUB_WARMUP_MS  = 2000;   // Wert durch Experimentieren gefunden
const uint32_t A01NYUB_TIMEOUT_MS = 3000;
const int      A01NYUB_MIN_MM     = 260;    // darunter liefert der Sensor keine gültigen Werte

// OLED
#define OLED_CLOCK 15
#define OLED_DATA  4
#define OLED_RESET 16


// ============================================================================
// MARK: - Globale Objekte
// ============================================================================

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

struct {
  int id;
  DeviceAddress addr;
} T[MAX_DS1820];
int countDS1820 = 0;

HardwareSerial sonarSerial(1);

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, OLED_CLOCK, OLED_DATA, OLED_RESET);
bool displayOn = true;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

#define MSG_BUFFER_SIZE 64
char msg[MSG_BUFFER_SIZE];
char topic[MSG_BUFFER_SIZE];

uint32_t lastDistanceMs     = 0;
uint32_t lastTemperatureMs  = 0;
uint32_t lastHeartbeatMs    = 0;
uint32_t lastMqttAttemptMs  = 0;
uint32_t lastSensorScanMs   = 0;
uint32_t lastPublishOkMs    = 0;
uint32_t wifiLostSinceMs    = 0;
bool     wifiWasConnected   = false;


// ============================================================================
// MARK: - Funktionsdeklarationen
// ============================================================================

void setupWatchdog();
void setupDisplay();
void setupWiFi();
void scanTemperatureSensors();
void handleWiFi();
void handleMqtt();
bool connectMqtt();
bool publish(const char* t, const char* payload, bool retained = false);
void callbackMqtt(char* t, byte* payload, unsigned int length);
void measureDistance();
void measureTemperatures();
void sendHeartbeat();
void checkHealth();
int  readA01NYUB();
bool readByte(uint8_t &b, uint32_t startMs);
void printAddress(DeviceAddress deviceAddress);
void safeRestart(const char* reason);

// überlauf-sicherer Vergleich: true, wenn seit 'since' mindestens 'interval' ms vergangen sind
inline bool elapsed(uint32_t since, uint32_t interval) {
  return (uint32_t)(millis() - since) >= interval;
}


// ============================================================================
// MARK: - Setup
// ============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("Booting");

  setupWatchdog();
  setupDisplay();

  // Sensor erst einmal stromlos
  pinMode(A01NYUB_VCC_PIN, OUTPUT);
  digitalWrite(A01NYUB_VCC_PIN, LOW);
  sonarSerial.begin(A01NYUB_BAUD, SERIAL_8N1, A01NYUB_RX_PIN, A01NYUB_TX_PIN);

  sensors.begin();
  scanTemperatureSensors();

  setupWiFi();

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  mqttClient.setServer(mqtt_server, MQTT_PORT);
  mqttClient.setCallback(callbackMqtt);
  mqttClient.setKeepAlive(30);
  mqttClient.setSocketTimeout(5);

  // erste Messungen ~10 s nach dem Start
  uint32_t now = millis();
  lastDistanceMs    = now - INTERVAL_DISTANCE_MS + 10000;
  lastTemperatureMs = now - INTERVAL_TEMPERATURE_MS + 10000;
  lastHeartbeatMs   = now - INTERVAL_HEARTBEAT_MS;
  lastMqttAttemptMs = now - INTERVAL_MQTT_RETRY_MS;
  lastSensorScanMs  = now;
  lastPublishOkMs   = now;
}


// ============================================================================
// MARK: - Hauptprogramm
// ============================================================================

void loop() {
  esp_task_wdt_reset();

  handleWiFi();
  handleMqtt();

  if (elapsed(lastDistanceMs, INTERVAL_DISTANCE_MS)) {
    lastDistanceMs = millis();
    measureDistance();
  }

  if (elapsed(lastTemperatureMs, INTERVAL_TEMPERATURE_MS)) {
    lastTemperatureMs = millis();
    measureTemperatures();
  }

  if (elapsed(lastHeartbeatMs, INTERVAL_HEARTBEAT_MS)) {
    lastHeartbeatMs = millis();
    sendHeartbeat();
  }

  checkHealth();
  delay(10);
}


// ============================================================================
// MARK: - Initialisierung
// ============================================================================

void setupWatchdog() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true,
  };
  esp_task_wdt_reconfigure(&cfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);
}

void setupDisplay() {
  u8g2.begin();
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_t0_17b_mf);
  u8g2.drawStr(10, 12, "Schneeniveau");
  u8g2.drawHLine(20, 16, 100);
  u8g2.sendBuffer();
}

void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.begin(SSID, WiFiPassword);

  // beim Start max. 20 s warten; danach läuft handleWiFi() weiter
  Serial.print("Connecting to WiFi");
  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && !elapsed(start, 20000)) {
    Serial.print('.');
    esp_task_wdt_reset();
    delay(500);
  }
  Serial.println();
  wifiLostSinceMs = millis();
}

void scanTemperatureSensors() {
  lastSensorScanMs = millis();
  sensors.begin();
  countDS1820 = sensors.getDeviceCount();
  if (countDS1820 > MAX_DS1820) countDS1820 = MAX_DS1820;
  Serial.printf("Found %d DS18B20 devices.\n", countDS1820);

  int found = 0;
  for (int index = 0; index < countDS1820; index++) {
    if (!sensors.getAddress(T[found].addr, index)) continue;
    T[found].id = sensors.getUserData(T[found].addr);
    sensors.setResolution(T[found].addr, TEMPERATURE_PRECISION);
    Serial.printf("\tIndex: %d\tID: %d\tAddr: ", index, T[found].id);
    printAddress(T[found].addr);
    Serial.println();
    found++;
  }
  countDS1820 = found;
}


// ============================================================================
// MARK: - Verbindungen
// ============================================================================

void handleWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      Serial.print("WiFi connected, IP: ");
      Serial.println(WiFi.localIP());
      if (displayOn) {
        u8g2.setFont(u8g2_font_t0_11_mf);
        u8g2.setCursor(0, 62);
        u8g2.print(WiFi.localIP());
        u8g2.sendBuffer();
      }
    }
    wifiLostSinceMs = millis();
    return;
  }

  if (wifiWasConnected) {
    wifiWasConnected = false;
    Serial.println("WiFi lost");
  }

  // Auto-Reconnect hilft nicht immer, daher zusätzlich manuell neu verbinden
  static uint32_t lastReconnectMs = 0;
  if (elapsed(lastReconnectMs, 30000)) {
    lastReconnectMs = millis();
    Serial.println("WiFi reconnect...");
    WiFi.disconnect();
    WiFi.begin(SSID, WiFiPassword);
  }

  if (elapsed(wifiLostSinceMs, WIFI_LOST_RESTART_MS)) {
    safeRestart("WiFi too long offline");
  }
}

void handleMqtt() {
  if (WiFi.status() != WL_CONNECTED) return;

  if (mqttClient.connected()) {
    mqttClient.loop();
    return;
  }

  if (elapsed(lastMqttAttemptMs, INTERVAL_MQTT_RETRY_MS)) {
    lastMqttAttemptMs = millis();
    connectMqtt();
  }
}

bool connectMqtt() {
  Serial.print("Attempting MQTT connection... ");
  esp_task_wdt_reset();

  if (mqttClient.connect(MQTT_CLIENT_ID, mqtt_user, mqtt_pass, TOPIC_STATUS, 0, true, MSG_OFFLINE)) {
    Serial.println("connected");
    publish(TOPIC_STATUS, MSG_ONLINE, true);
    mqttClient.subscribe(TOPIC_SUBSCRIBE);
    return true;
  }

  Serial.printf("failed, rc=%d\n", mqttClient.state());
  return false;
}

bool publish(const char* t, const char* payload, bool retained) {
  if (!mqttClient.connected()) return false;
  bool ok = mqttClient.publish(t, payload, retained);
  if (ok) lastPublishOkMs = millis();
  return ok;
}

void callbackMqtt(char* t, byte* payload, unsigned int length) {
  Serial.printf("Message arrived [%s] ", t);
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  // "restart" auf gruben/sub/cmd startet die Station aus der Ferne neu
  if (strcmp(t, "gruben/sub/cmd") == 0 && length == 7 && memcmp(payload, "restart", 7) == 0) {
    safeRestart("remote command");
  }
}


// ============================================================================
// MARK: - Messungen
// ============================================================================

void measureDistance() {
  int distance = readA01NYUB();

  if (displayOn) {
    u8g2.setFont(u8g2_font_t0_17b_mf);
    u8g2.setCursor(0, 34);
    u8g2.print("Snow:         ");
    u8g2.setCursor(60, 34);
  }

  if (distance > 0) {
    Serial.printf("Distance measured: %d mm\n", distance);
    snprintf(msg, MSG_BUFFER_SIZE, "%d", distance);
    publish(TOPIC_DISTANCE, msg);
    if (displayOn) {
      u8g2.print(distance);
      u8g2.print("mm");
    }
  } else {
    Serial.println("ERROR measuring distance");
    if (displayOn) u8g2.print("N/A");
  }

  if (displayOn) u8g2.sendBuffer();
}

void measureTemperatures() {
  if (countDS1820 == 0) {
    if (elapsed(lastSensorScanMs, INTERVAL_SENSOR_SCAN_MS)) scanTemperatureSensors();
    return;
  }

  sensors.requestTemperatures();   // blockiert max. ~750 ms

  for (int index = 0; index < countDS1820; index++) {
    float tempC = sensors.getTempC(T[index].addr);
    if (tempC == DEVICE_DISCONNECTED_C || tempC == 85.0f) {
      Serial.printf("Sensor %d: read error\n", T[index].id);
      continue;
    }
    Serial.printf("Sensor %d: %.1f C\n", T[index].id, tempC);
    snprintf(msg, MSG_BUFFER_SIZE, "%.1f", tempC);
    snprintf(topic, MSG_BUFFER_SIZE, TOPIC_TEMP_FMT, T[index].id);
    publish(topic, msg);
  }

  struct tm timeInfo;
  if (displayOn && getLocalTime(&timeInfo, 10)) {
    u8g2.setFont(u8g2_font_t0_11_mf);
    u8g2.setCursor(0, 52);
    u8g2.print(&timeInfo, "%d %b %Y %H:%M");
    u8g2.sendBuffer();
  }
}

void sendHeartbeat() {
  snprintf(msg, MSG_BUFFER_SIZE, "%lu", (unsigned long)(esp_timer_get_time() / 1000000ULL));
  publish(TOPIC_UPTIME, msg);
  snprintf(msg, MSG_BUFFER_SIZE, "%d", WiFi.RSSI());
  publish(TOPIC_RSSI, msg);
}


// ============================================================================
// MARK: - Überwachung
// ============================================================================

void checkHealth() {
  if (displayOn && elapsed(0, DISPLAY_ON_MS)) {
    displayOn = false;
    u8g2.setPowerSave(1);
  }

  if (elapsed(lastPublishOkMs, NO_PUBLISH_RESTART_MS)) {
    safeRestart("nothing published for too long");
  }

  if (ESP.getFreeHeap() < MIN_FREE_HEAP) {
    safeRestart("low heap");
  }

  // esp_timer läuft 64 bit und läuft nicht über
  if (esp_timer_get_time() / 1000ULL > PREVENTIVE_RESTART_MS) {
    safeRestart("preventive restart");
  }
}

void safeRestart(const char* reason) {
  Serial.printf("Restarting: %s\n", reason);
  if (mqttClient.connected()) {
    mqttClient.publish(TOPIC_STATUS, reason, false);
    mqttClient.loop();
    mqttClient.disconnect();
  }
  digitalWrite(A01NYUB_VCC_PIN, LOW);
  delay(500);
  ESP.restart();
}


// ============================================================================
// MARK: - A01NYUB
// ============================================================================

// liest ein Byte mit Gesamt-Timeout ab 'startMs'
bool readByte(uint8_t &b, uint32_t startMs) {
  while (!sonarSerial.available()) {
    if (elapsed(startMs, A01NYUB_TIMEOUT_MS)) return false;
    delay(1);
  }
  b = sonarSerial.read();
  return true;
}

// Datenformat: 0xFF, High, Low, Checksumme
// siehe https://wiki.dfrobot.com/A01NYUB%20Waterproof%20Ultrasonic%20Sensor%20SKU:%20SEN0313
int readA01NYUB() {
  digitalWrite(A01NYUB_VCC_PIN, HIGH);

  uint32_t warmup = millis();
  while (!elapsed(warmup, A01NYUB_WARMUP_MS)) {
    if (mqttClient.connected()) mqttClient.loop();
    delay(10);
  }

  // alte Daten verwerfen
  while (sonarSerial.available() > 0) sonarSerial.read();

  int result = 0;
  uint32_t start = millis();
  uint8_t b, high, low, sum;

  // mehrere Versuche innerhalb des Timeouts, bis ein gültiges Paket kommt
  while (!elapsed(start, A01NYUB_TIMEOUT_MS)) {
    if (!readByte(b, start)) break;
    if (b != 0xFF) continue;                       // auf Header synchronisieren
    if (!readByte(high, start)) break;
    if (high == 0xFF) continue;                    // doppelter Header / Padding
    if (!readByte(low, start)) break;
    if (!readByte(sum, start)) break;

    if (((0xFF + high + low) & 0xFF) != sum) {
      Serial.println("Checksum Error");
      continue;
    }

    int distance = (high << 8) + low;
    if (distance > A01NYUB_MIN_MM) {
      result = distance;
      break;
    }
    Serial.println("Below the lower limit");
  }

  digitalWrite(A01NYUB_VCC_PIN, LOW);
  return result;
}


// ============================================================================
// MARK: - Hilfsfunktionen
// ============================================================================

void printAddress(DeviceAddress deviceAddress) {
  for (uint8_t i = 0; i < 8; i++) {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
