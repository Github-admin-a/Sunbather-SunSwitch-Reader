/**************************************************************
   SunSwitch Reader + MQTT + Web Interface + OTA   (V3.12)
   Target board: Wemos D1 Mini (ESP8266)

   V3.12 - Glitch filtering. A value is only published once two
           good frames in a row agree on it, so a single frame
           damaged in transit (roof 142/200, "Unknown" mode,
           1-second pump OFF) never reaches Home Assistant.
           Roof temps above 99 reject the frame, runtime must
           tick forward sensibly, and the main page shows how
           many glitches were caught.

   V3.11 - Roof temperature and pump status go unavailable in
           Home Assistant while the mode is "Offline".
   V3.10 - Controller mode shows "Offline" when the SunSwitch /
           Wemos is powered off or stops sending frames.
   V3.9 - IP address, Wemos uptime and sensor data error are
          published to MQTT / Home Assistant again.
   V3.8 - Energy Saving shown in controller mode (byte 8),
          WiFi auto-reconnect / self-restart, no more
          hanging in the WiFi portal after a power cut.

   Command Scanner (web page: /scanner)
     Sends ONE candidate command, waits (default 5 s), then
     checks whether set temp (byte 10), mode (byte 14) or pump
     (byte 15) changed. Anything that works is saved as a "hit"
     and can be re-sent from the web page.
       Phase 1: every single byte 00..FF
       Phase 2: "Sun" frames with a new set temp / mode
       Phase 3: "Sun" + every byte 00..FF
       Phase 4: every byte 00..FF followed by CR
       Phase 5: letter + new set temp (binary and text forms)
     Optional "send first" command (e.g. 52 = enter Set Temp
     Mode) is sent before each candidate when needed.
     Also captures any non-frame bytes the controller sends
     (replies, start-up text) and detects controller restarts.
     Status frames are now validated (must end 0A 0D, sane
     values) so reception errors are not reported as hits.
**************************************************************/

/**************************************************************
   SunSwitch Reader + MQTT + Web Interface + OTA
   ------------------------------------------------------------
   Purpose:
     Reads live telemetry from a SunSwitch pool controller via
     its serial diagnostic port, decodes the 23‑byte status
     frame, and exposes values to Home Assistant via MQTT.

   Hardware:
     - Wemos D1 Mini (ESP8266)
     - Connected to SunSwitch UART via SoftwareSerial
     - Powered from SunSwitch 5V rail or external supply

   Features:
   ------------------------------------------------------------
   • Real‑time SunSwitch frame decoding
       - Reads 23‑byte "Sun" frame at 57,600 baud
       - Extracts:
           • Set temperature
           • Pool sensor 1 temperature
           • Pool sensor 2 temperature
           • Roof temperature
           • Pump status (byte 15: 0=off, 1=on)
           • Controller mode (byte 14: 09=auto, 17=manual)
           • Runtime (seconds)
           • Raw frame bytes for debugging

   • MQTT publishing (with username + password)
       - Publishes decoded values to user‑defined topics
           • sunswitch/set_temp
           • sunswitch/pool_temp
           • sunswitch/roof_temp
           • sunswitch/pump_status
           • sunswitch/controller_mode
           • sunswitch/pool_error (reserved / future use)
       - Supports retained messages
       - Supports Home Assistant auto‑discovery
       - Supports Last Will & Testament (LWT)
           • "sunswitch/status" → online/offline

   • Home Assistant auto‑discovery
       - Creates sensors automatically:
           • sunswitch_set_temp        (temperature)
           • sunswitch_pool_temp       (temperature)
           • sunswitch_roof_temp       (temperature)
           • sunswitch_pump_status     (binary_sensor)
           • sunswitch_controller_mode (enum sensor)
       - Runtime is still published to MQTT but not
         auto‑discovered as a HA sensor.
       - Works with Mosquitto add‑on
       - No YAML required

   • Web interface (fully styled)
       - Controller status & sensor display
       - WiFi status + signal quality
       - Raw frame viewer
       - Debug byte table (with function column)
       - Pool sensor selector (1 or 2)
       - Command sender (supports \r and \n)
       - MQTT configuration page
           • MQTT connection status
           • HA discovery status + manual trigger
       - Firmware update page (HTTP OTA)
       - WiFi setup page (launches WiFiManager portal)
       - Command scanner page (helper)

   • WiFiManager integration
       - AutoConnect on boot
       - Captive portal for first‑time setup
       - Manual portal launch from web UI

   • OTA firmware updates
       - ArduinoOTA enabled (no password) - use 1234 if it asks
       - HTTP OTA upload via /update

   • SPIFFS configuration storage
       - mqtt.cfg
           • host
           • port
           • username
           • password
           • set_temp topic
           • pool_temp topic
           • roof_temp topic
           • pump_status topic
           • error topic (reserved)
           • controller_mode topic
       - poolsel.cfg
           • selected pool sensor (1 or 2)

   • Robustness
       - MQTT reconnect backoff
       - Birth + LWT messages
       - Safe frame parsing state machine
       - No blocking delays
       - OTA + web server + MQTT all coexist cleanly

**************************************************************/

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <FS.h>
#include <SoftwareSerial.h>
#include <ArduinoOTA.h>
#include <ESP8266HTTPUpdateServer.h>

// ------------------------------------------------------------
// SunSwitch UART via SoftwareSerial
// ------------------------------------------------------------
SoftwareSerial sunSerial(D1, D2); // RX=D1, TX=D2

// ------------------------------------------------------------
// Web server + OTA update + MQTT client
// ------------------------------------------------------------
ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;
WiFiClient espClient;
PubSubClient mqtt(espClient);

// Second MQTT connection. It publishes nothing; it exists only so the
// broker can set the controller-mode topic to "Offline" (its Last Will)
// when the Wemos loses power or drops off the network. The main
// connection's Last Will is already used for sunswitch/status.
WiFiClient   espClientWill;
PubSubClient mqttWill(espClientWill);
unsigned long lastWillAttempt = 0;

// ------------------------------------------------------------
// MQTT configuration (stored in SPIFFS)
// ------------------------------------------------------------
String   mqttHost = "";
uint16_t mqttPort = 1883;
String   mqttUser = "";
String   mqttPass = "";

// MQTT topics (defaults; can be overridden via web UI)
String mqttTopicSet    = "sunswitch/set_temp";
String mqttTopicPool   = "sunswitch/pool_temp";
String mqttTopicRoof   = "sunswitch/roof_temp";
String mqttTopicPump   = "sunswitch/pump_status";
String mqttTopicError  = "sunswitch/pool_error";      // reserved / future use
String mqttTopicMode   = "sunswitch/controller_mode"; // controller mode
const char *mqttTopicStatus = "sunswitch/status";       // LWT / availability
const char *mqttTopicIp     = "sunswitch/ip_address";   // diagnostic: current IP address
const char *mqttTopicUptime = "sunswitch/esp_uptime";   // diagnostic: time since the Wemos booted

unsigned long lastMqttAttempt = 0;

// ------------------------------------------------------------
// MQTT / Home Assistant discovery status
// ------------------------------------------------------------
bool          discoveryPublished     = false;
String        discoveryStatus        = "Not published yet.";
unsigned long lastDiscoveryAttempt   = 0;
int           discoverySuccessCount  = 0;
int           discoveryFailCount     = 0;

// ------------------------------------------------------------
// Pool sensor selection (1 or 2), stored in SPIFFS
// ------------------------------------------------------------
int selectedPoolSensor = 1;

// Load selected pool sensor from SPIFFS
void loadPoolSelection() {
  if (!SPIFFS.begin()) return;
  if (!SPIFFS.exists("/poolsel.cfg")) return;

  File f = SPIFFS.open("/poolsel.cfg", "r");
  if (!f) return;

  selectedPoolSensor = f.readStringUntil('\n').toInt();
  if (selectedPoolSensor != 1 && selectedPoolSensor != 2)
    selectedPoolSensor = 1;

  f.close();
}

// Save selected pool sensor to SPIFFS
void savePoolSelection() {
  if (!SPIFFS.begin()) return;

  File f = SPIFFS.open("/poolsel.cfg", "w");
  if (!f) return;

  f.println(selectedPoolSensor);
  f.close();
}

// ------------------------------------------------------------
// SunSwitch frame storage (23 bytes)
// ------------------------------------------------------------
uint8_t frame[23];
bool    frameValid = false;
String  lastFrameHex;

// Decoded values
float   setTempC          = 0;
float   poolTemp1C        = 0;
float   poolTemp2C        = 0;
float   roofTempC         = 0;
uint8_t pumpStatus        = 0;   // byte 15: 0 = off, 1 = on
uint8_t controllerMode    = 0;   // byte 14: 09 = auto, 17 = manual
String  controllerModeStr = "Unknown";

// ------------------------------------------------------------
// V3.12 glitch filter
// Each value is only accepted when two good frames in a row agree
// on it. A single frame damaged in transit (one byte flipped or
// bit-slipped by SoftwareSerial) therefore can't change anything.
// ------------------------------------------------------------
uint8_t       prevRx[23];                 // previous good frame, as received
bool          havePrevRx      = false;
unsigned long prevRxMs        = 0;
const unsigned long CONFIRM_MAX_GAP_MS = 10000UL;  // frames further apart don't confirm each other

// Set true for the frame in which each value was confirmed
bool newSet = false, newPool1 = false, newPool2 = false, newRoof = false,
     newPump = false, newMode = false, newRuntime = false;
bool haveConfirmed = false;               // at least one value confirmed since boot
uint16_t confirmedRuntime = 0;
uint32_t glitchesFiltered = 0;            // frames where a value was held back for a second look

// ------------------------------------------------------------
// Helper: byte to hex string
// ------------------------------------------------------------
String toHex(uint8_t b) {
  char buf[4];
  sprintf(buf, "%02X", b);
  return String(buf);
}

// ------------------------------------------------------------
// Helper: WiFi quality description
// ------------------------------------------------------------
String wifiQuality(int rssi) {
  if (rssi >= -50) return "Excellent";
  if (rssi >= -60) return "Very Good";
  if (rssi >= -67) return "Good";
  if (rssi >= -75) return "Fair";
  if (rssi >= -85) return "Weak";
  return "Very Poor";
}

// ------------------------------------------------------------
// WiFi watchdog: if the connection drops, retry every 30 s and
// restart the Wemos if it has been down for 10 minutes.
// ------------------------------------------------------------
unsigned long wifiDownSince = 0;
unsigned long lastWifiRetry = 0;
uint32_t      wifiDropCount = 0;

void wifiWatchdog() {
  if (WiFi.status() == WL_CONNECTED) {
    wifiDownSince = 0;
    return;
  }

  unsigned long now = millis();
  if (wifiDownSince == 0) {
    wifiDownSince = now ? now : 1;
    lastWifiRetry = now;
    wifiDropCount++;
    return;
  }
  if (now - lastWifiRetry >= 30000UL) {
    lastWifiRetry = now;
    WiFi.reconnect();
  }
  if (now - wifiDownSince >= 600000UL) {
    ESP.restart();
  }
}

// ------------------------------------------------------------
// Build hex representation of last frame
// ------------------------------------------------------------
void buildFrameHex() {
  lastFrameHex = "";
  for (int i = 0; i < 23; i++) {
    if (i) lastFrameHex += " ";
    lastFrameHex += toHex(frame[i]);
  }
}

// ------------------------------------------------------------
// Controller mode name for a mode byte (byte 14):
//   0x08 = Controller off     0x09 = Automatic
//   0x11 = Manual             0x03 = Winter Mode
//   0x49 = Set Temp Mode      0x91 = Continuous Manual
// ------------------------------------------------------------
String modeNameFor(uint8_t m) {
  switch (m) {
    case 0x08: return "Controller Off";
    case 0x09: return "Automatic";
    case 0x11: return "Manual";
    case 0x03: return "Winter Mode";
    case 0x49: return "Set Temp Mode";
    case 0x91: return "Continuous Manual";
    default:   return "Unknown";
  }
}

// ------------------------------------------------------------
// Decode temperatures + pump + controller mode from frame bytes.
// Only values that match the previous good frame are accepted.
// ------------------------------------------------------------
void updateTempsFromFrame() {
  newSet = newPool1 = newPool2 = newRoof = newPump = newMode = newRuntime = false;
  if (!frameValid) return;

  unsigned long now = millis();
  bool canConfirm = havePrevRx && (now - prevRxMs <= CONFIRM_MAX_GAP_MS);

  if (canConfirm) {
    // Temperatures + pump: same byte in both frames
    if (frame[10] == prevRx[10]) { setTempC   = frame[10]; newSet   = true; }
    if (frame[11] == prevRx[11]) { poolTemp1C = frame[11]; newPool1 = true; }
    if (frame[12] == prevRx[12]) { poolTemp2C = frame[12]; newPool2 = true; }
    if (frame[13] == prevRx[13]) { roofTempC  = frame[13]; newRoof  = true; }
    if (frame[15] == prevRx[15]) { pumpStatus = frame[15]; newPump  = true; }

    // Mode: same mode byte AND same energy-saving state. Byte 8 is a
    // timer that counts, so only compare whether it is running.
    bool esNow  = frame[8]  > 0x00;
    bool esPrev = prevRx[8] > 0x00;
    if (frame[14] == prevRx[14] && esNow == esPrev) {
      controllerMode    = frame[14];
      // Byte 8 is the energy-saving timer: above zero while the controller
      // is in Energy Saving ("ES" on its display), 0x00 otherwise.
      controllerModeStr = modeNameFor(controllerMode) + (esNow ? " (Energy Saving)" : "");
      newMode = true;
    }

    // Runtime (bytes 4-5) counts up, so it must move forward a little
    uint16_t rtNow  = ((uint16_t)frame[4]  << 8) | frame[5];
    uint16_t rtPrev = ((uint16_t)prevRx[4] << 8) | prevRx[5];
    if ((uint16_t)(rtNow - rtPrev) <= 30) { confirmedRuntime = rtNow; newRuntime = true; }

    // Count frames where any value was held back (a glitch, or a real
    // change that will be confirmed by the next frame)
    if (!newRoof || !newMode || !newPump || !newSet || !newPool1 || !newPool2) glitchesFiltered++;

    if (newSet || newPool1 || newPool2 || newRoof || newPump || newMode) haveConfirmed = true;
  }

  memcpy(prevRx, frame, 23);
  havePrevRx = true;
  prevRxMs   = now;
}

// ------------------------------------------------------------
// Decode runtime from bytes 4 and 5 (seconds)
// ------------------------------------------------------------
String getRuntimeString() {
  if (!haveConfirmed) return "N/A";

  uint16_t seconds = confirmedRuntime;

  uint16_t s = seconds % 60;
  uint16_t m = (seconds / 60) % 60;
  uint16_t h = seconds / 3600;

  char buf[32];
  sprintf(buf, "%02u:%02u:%02u (from bytes 4:5)", h, m, s);
  return String(buf);
}

// ------------------------------------------------------------
// Load MQTT config from SPIFFS
// ------------------------------------------------------------
void loadMqttConfig() {
  if (!SPIFFS.begin()) return;
  if (!SPIFFS.exists("/mqtt.cfg")) return;

  File f = SPIFFS.open("/mqtt.cfg", "r");
  if (!f) return;

  mqttHost       = f.readStringUntil('\n'); mqttHost.trim();
  mqttPort       = f.readStringUntil('\n').toInt();
  mqttUser       = f.readStringUntil('\n'); mqttUser.trim();
  mqttPass       = f.readStringUntil('\n'); mqttPass.trim();
  mqttTopicSet   = f.readStringUntil('\n'); mqttTopicSet.trim();
  mqttTopicPool  = f.readStringUntil('\n'); mqttTopicPool.trim();
  mqttTopicRoof  = f.readStringUntil('\n'); mqttTopicRoof.trim();
  mqttTopicPump  = f.readStringUntil('\n'); mqttTopicPump.trim();
  mqttTopicError = f.readStringUntil('\n'); mqttTopicError.trim();
  mqttTopicMode  = f.readStringUntil('\n'); mqttTopicMode.trim();

  f.close();
}

// ------------------------------------------------------------
// Save MQTT config to SPIFFS
// ------------------------------------------------------------
void saveMqttConfig() {
  if (!SPIFFS.begin()) return;

  File f = SPIFFS.open("/mqtt.cfg", "w");
  if (!f) return;

  f.println(mqttHost);
  f.println(mqttPort);
  f.println(mqttUser);
  f.println(mqttPass);
  f.println(mqttTopicSet);
  f.println(mqttTopicPool);
  f.println(mqttTopicRoof);
  f.println(mqttTopicPump);
  f.println(mqttTopicError);
  f.println(mqttTopicMode);

  f.close();
}

// ------------------------------------------------------------
// Home Assistant MQTT discovery payloads
// ------------------------------------------------------------
// NOTE: Runtime is still published to MQTT but is not
//       auto‑discovered as a HA sensor. Pump status is exposed
//       as a binary_sensor, controller mode as an enum sensor.
// ------------------------------------------------------------
void publishDiscovery() {
  if (!mqtt.connected()) {
    discoveryPublished = false;
    discoveryStatus = "MQTT is not connected.";
    return;
  }

  lastDiscoveryAttempt = millis();

  // Home Assistant MQTT discovery expects JSON configuration on the
  // homeassistant/<component>/<object_id>/config topic.
  // All messages are retained so HA can discover the device even if
  // the Wemos is not online when Home Assistant starts.

  const char *deviceBlock =
    "\"device\":{"
      "\"identifiers\":[\"sunswitch_controller\"],"
      "\"name\":\"SunSwitch Controller\","
      "\"manufacturer\":\"SunSwitch\","
      "\"model\":\"SunSwitch Reader\""
    "}";

  bool allOK = true;
  String payload;

  // Roof temperature and pump status follow the controller-mode topic:
  // whenever the mode reads "Offline" (Wemos powered off, or controller
  // silent) Home Assistant marks them unavailable. Set temp and pool
  // temp deliberately keep their last value.
  String offlineAvail =
    String("\"availability_topic\":\"") + mqttTopicMode + "\"," +
    "\"availability_template\":\"{{ 'offline' if value == 'Offline' else 'online' }}\",";

  // ----------------------------------------------------------
  // Set temperature (sensor)
  // ----------------------------------------------------------
  payload =
    String("{\"name\":\"SunSwitch Set Temperature\",") +
    "\"state_topic\":\"" + mqttTopicSet + "\"," +
    "\"unit_of_measurement\":\"°C\"," +
    "\"device_class\":\"temperature\"," +
    "\"state_class\":\"measurement\"," +
    "\"unique_id\":\"sunswitch_set_temp\"," +
    deviceBlock + "}";

  bool ok = mqtt.publish(
    "homeassistant/sensor/sunswitch_set_temp/config",
    payload.c_str(),
    true
  );
  if (!ok) allOK = false;

  // ----------------------------------------------------------
  // Pool temperature (sensor)
  // ----------------------------------------------------------
  payload =
    String("{\"name\":\"SunSwitch Pool Temperature\",") +
    "\"state_topic\":\"" + mqttTopicPool + "\"," +
    "\"unit_of_measurement\":\"°C\"," +
    "\"device_class\":\"temperature\"," +
    "\"state_class\":\"measurement\"," +
    "\"unique_id\":\"sunswitch_pool_temp\"," +
    deviceBlock + "}";

  ok = mqtt.publish(
    "homeassistant/sensor/sunswitch_pool_temp/config",
    payload.c_str(),
    true
  );
  if (!ok) allOK = false;

  // ----------------------------------------------------------
  // Roof temperature (sensor)
  // ----------------------------------------------------------
  payload =
    String("{\"name\":\"SunSwitch Roof Temperature\",") +
    "\"state_topic\":\"" + mqttTopicRoof + "\"," +
    offlineAvail +
    "\"unit_of_measurement\":\"°C\"," +
    "\"device_class\":\"temperature\"," +
    "\"state_class\":\"measurement\"," +
    "\"unique_id\":\"sunswitch_roof_temp\"," +
    deviceBlock + "}";

  ok = mqtt.publish(
    "homeassistant/sensor/sunswitch_roof_temp/config",
    payload.c_str(),
    true
  );
  if (!ok) allOK = false;

  // ----------------------------------------------------------
  // Pump status (binary_sensor)
  // ----------------------------------------------------------
  payload =
    String("{\"name\":\"SunSwitch Pump Status\",") +
    "\"state_topic\":\"" + mqttTopicPump + "\"," +
    offlineAvail +
    "\"device_class\":\"power\"," +
    "\"payload_on\":\"ON\"," +
    "\"payload_off\":\"OFF\"," +
    "\"unique_id\":\"sunswitch_pump_status\"," +
    deviceBlock + "}";

  ok = mqtt.publish(
    "homeassistant/binary_sensor/sunswitch_pump_status/config",
    payload.c_str(),
    true
  );
  if (!ok) allOK = false;

  // ----------------------------------------------------------
  // Controller mode (enum sensor)
  // ----------------------------------------------------------
  payload =
    String("{\"name\":\"SunSwitch Controller Mode\",") +
    "\"state_topic\":\"" + mqttTopicMode + "\"," +
    "\"device_class\":\"enum\"," +
    "\"options\":[\"Automatic\",\"Manual\",\"Winter Mode\",\"Set Temp Mode\",\"Continuous Manual\",\"Controller Off\",\"Unknown\"," +
    "\"Automatic (Energy Saving)\",\"Manual (Energy Saving)\",\"Winter Mode (Energy Saving)\",\"Set Temp Mode (Energy Saving)\",\"Continuous Manual (Energy Saving)\",\"Controller Off (Energy Saving)\",\"Unknown (Energy Saving)\",\"Offline\"]," +
    "\"unique_id\":\"sunswitch_controller_mode\"," +
    deviceBlock + "}";

  ok = mqtt.publish(
    "homeassistant/sensor/sunswitch_controller_mode/config",
    payload.c_str(),
    true
  );
  if (!ok) allOK = false;

  // ----------------------------------------------------------
  // Diagnostics: sensor data error, IP address, Wemos uptime.
  // These go unavailable in HA when the Wemos is offline or has
  // not refreshed them for 5 minutes.
  // ----------------------------------------------------------
  String availability =
    String("\"availability_topic\":\"") + mqttTopicStatus + "\"," +
    "\"payload_available\":\"online\"," +
    "\"payload_not_available\":\"offline\"," +
    "\"expire_after\":300,";

  payload =
    String("{\"name\":\"SunSwitch Sensor Data Error\",") +
    "\"state_topic\":\"" + mqttTopicError + "\"," +
    availability +
    "\"device_class\":\"problem\"," +
    "\"payload_on\":\"ON\"," +
    "\"payload_off\":\"OFF\"," +
    "\"unique_id\":\"sunswitch_sensor_error\"," +
    deviceBlock + "}";
  if (!mqtt.publish("homeassistant/binary_sensor/sunswitch_sensor_error/config", payload.c_str(), true)) allOK = false;

  payload =
    String("{\"name\":\"SunSwitch IP Address\",") +
    "\"state_topic\":\"" + mqttTopicIp + "\"," +
    availability +
    "\"icon\":\"mdi:ip-network\"," +
    "\"entity_category\":\"diagnostic\"," +
    "\"unique_id\":\"sunswitch_ip_address\"," +
    deviceBlock + "}";
  if (!mqtt.publish("homeassistant/sensor/sunswitch_ip_address/config", payload.c_str(), true)) allOK = false;

  payload =
    String("{\"name\":\"SunSwitch ESP Uptime\",") +
    "\"state_topic\":\"" + mqttTopicUptime + "\"," +
    availability +
    "\"icon\":\"mdi:clock-outline\"," +
    "\"entity_category\":\"diagnostic\"," +
    "\"unique_id\":\"sunswitch_esp_uptime\"," +
    deviceBlock + "}";
  if (!mqtt.publish("homeassistant/sensor/sunswitch_esp_uptime/config", payload.c_str(), true)) allOK = false;

  if (allOK) {
    discoveryPublished = true;
    discoverySuccessCount++;
    discoveryStatus = "Home Assistant discovery messages published (set, pool, roof, pump, mode, error, IP, uptime).";
  } else {
    discoveryPublished = false;
    discoveryFailCount++;
    discoveryStatus = "Discovery publish failed. Check MQTT packet size/connection.";
  }
}

// ------------------------------------------------------------
// Publish decoded values to MQTT
// ------------------------------------------------------------
void mqttPublish() {
  if (!frameValid || !mqtt.connected()) return;

  // Only values confirmed by this frame (two good frames agree) are sent.

  // Set temperature
  if (newSet) mqtt.publish(mqttTopicSet.c_str(), String(setTempC, 0).c_str(), true);

  // Selected pool sensor (1 or 2)
  if (selectedPoolSensor == 1 ? newPool1 : newPool2) {
    float poolToSend = (selectedPoolSensor == 1 ? poolTemp1C : poolTemp2C);
    mqtt.publish(mqttTopicPool.c_str(), String(poolToSend, 0).c_str(), true);
  }

  // Roof temperature
  if (newRoof) mqtt.publish(mqttTopicRoof.c_str(), String(roofTempC, 0).c_str(), true);

  // Pump status: publish "ON" / "OFF" to mqttTopicPump
  if (newPump) mqtt.publish(mqttTopicPump.c_str(), pumpStatus == 1 ? "ON" : "OFF", true);

  // Controller mode string
  if (newMode) mqtt.publish(mqttTopicMode.c_str(), controllerModeStr.c_str(), true);

  // Runtime in seconds (still published, but not auto-discovered)
  if (newRuntime) mqtt.publish("sunswitch/runtime", String(confirmedRuntime).c_str(), true);

  // Pool error topic is reserved; you can later map a specific
  // frame byte to an error code and publish here if desired.
}

// ------------------------------------------------------------
// Diagnostics: IP address, Wemos uptime, sensor data error.
// Published on connect and then every 30 s (see loop()).
// ------------------------------------------------------------
uint8_t consecutiveBadFrames = 0;   // garbled frames in a row (3+ = sensor data error)

// "Offline" handling for the controller mode
unsigned long lastValidFrameMs     = 0;      // when the last good status frame arrived
bool          modeOfflinePublished = false;  // "Offline" already sent for this outage

void mqttWillReconnect() {
  if (mqttHost.length() == 0) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (!mqtt.connected()) return;               // main connection first
  if (mqttWill.connected()) return;

  unsigned long now = millis();
  if (now - lastWillAttempt < 10000) return;
  lastWillAttempt = now;

  mqttWill.setServer(mqttHost.c_str(), mqttPort);
  mqttWill.setSocketTimeout(5);

  bool auth = mqttUser.length() > 0;
  mqttWill.connect(
    "SunSwitchWemosMode",
    auth ? mqttUser.c_str() : NULL,
    auth ? mqttPass.c_str() : NULL,
    mqttTopicMode.c_str(),
    1,
    true,
    "Offline"
  );
}

// Wemos still running but the controller has gone quiet (no good
// frame for 60 s): report the mode as "Offline" ourselves.
void checkControllerOffline() {
  if (modeOfflinePublished) return;
  if (millis() - lastValidFrameMs < 60000UL) return;
  if (!mqtt.connected()) return;

  controllerModeStr = "Offline";
  mqtt.publish(mqttTopicMode.c_str(), "Offline", true);
  modeOfflinePublished = true;
}

String espUptimeString() {
  unsigned long t = millis() / 1000;
  char buf[32];
  sprintf(buf, "%lud %02lu:%02lu:%02lu", t / 86400, (t / 3600) % 24, (t / 60) % 60, t % 60);
  return String(buf);
}

void publishDiagnostics() {
  if (!mqtt.connected()) return;
  mqtt.publish(mqttTopicStatus, "online", true);
  mqtt.publish(mqttTopicIp, WiFi.localIP().toString().c_str(), true);
  mqtt.publish(mqttTopicUptime, espUptimeString().c_str(), true);
  mqtt.publish(mqttTopicError.c_str(), consecutiveBadFrames >= 3 ? "ON" : "OFF", true);
}

// ------------------------------------------------------------
// MQTT reconnect with LWT and optional auth
// ------------------------------------------------------------
void mqttReconnect() {
  if (mqttHost.length() == 0) {
    discoveryStatus = "MQTT host not configured.";
    return;
  }

  if (mqtt.connected()) return;
  if (WiFi.status() != WL_CONNECTED) return;   // no point trying without WiFi

  unsigned long now = millis();
  if (now - lastMqttAttempt < 10000) return;

  lastMqttAttempt = now;

  mqtt.setServer(mqttHost.c_str(), mqttPort);
  mqtt.setBufferSize(1024);   // room for the longer controller-mode option list
  mqtt.setSocketTimeout(5);   // don't stall the loop for long if the broker is unreachable

  bool ok;

  if (mqttUser.length() > 0) {
    ok = mqtt.connect(
      "SunSwitchWemos",
      mqttUser.c_str(),
      mqttPass.c_str(),
      "sunswitch/status",
      1,
      true,
      "offline"
    );
  } else {
    ok = mqtt.connect(
      "SunSwitchWemos",
      NULL,
      NULL,
      "sunswitch/status",
      1,
      true,
      "offline"
    );
  }

  if (ok) {
    mqtt.publish("sunswitch/status", "online", true);
    discoveryStatus = "MQTT connected. Publishing Home Assistant discovery...";
    publishDiscovery();
    publishDiagnostics();
  } else {
    discoveryPublished = false;
    discoveryStatus = "MQTT connection failed. PubSubClient state = " + String(mqtt.state());
  }
}

// ------------------------------------------------------------
// SunSwitch frame parser (looks for "Sun" + 20 bytes)
// ------------------------------------------------------------
void scanOnFrame();   // defined in the command scanner section
void scanStrayByte(uint8_t b);

// Frames are received into rxFrame and only copied to frame[] once
// they pass a sanity check, so a garbled frame never reaches the
// decoder, MQTT or the command scanner.
uint8_t  rxFrame[23];
uint32_t scanBadFrames = 0;

bool frameLooksValid(const uint8_t *f) {
  if (f[21] != 0x0A || f[22] != 0x0D) return false;   // every frame ends LF CR
  if (f[15] > 1) return false;                        // pump is 0 or 1
  if (f[10] > 60 || f[11] > 60 || f[12] > 60) return false;  // set / pool temps
  if (f[13] > 99) return false;                       // roof temp (V3.12)
  return true;
}

enum FrameState { WAIT_S, WAIT_U, WAIT_N, READ_BYTES };
FrameState fState = WAIT_S;
int        fIndex = 0;

// Read and parse frames from SunSwitch UART
void readSunSwitchFrame() {
  while (sunSerial.available()) {
    uint8_t b = sunSerial.read();

    switch (fState) {
      case WAIT_S:
        if (b == 0x53) { // 'S'
          rxFrame[0] = b;
          fState = WAIT_U;
        } else {
          scanStrayByte(b);          // not part of a frame: keep for the scanner
        }
        break;

      case WAIT_U:
        if (b == 0x75) { // 'u'
          rxFrame[1] = b;
          fState = WAIT_N;
        } else {
          scanStrayByte(0x53); scanStrayByte(b);
          fState = WAIT_S;
        }
        break;

      case WAIT_N:
        if (b == 0x6E) { // 'n'
          rxFrame[2] = b;
          fState = READ_BYTES;
          fIndex = 3;
        } else {
          scanStrayByte(0x53); scanStrayByte(0x75); scanStrayByte(b);
          fState = WAIT_S;
        }
        break;

      case READ_BYTES:
        rxFrame[fIndex++] = b;

        if (fIndex >= 23) {
          // Full frame received
          fState = WAIT_S;

          if (frameLooksValid(rxFrame)) {
            memcpy(frame, rxFrame, 23);
            frameValid = true;
            consecutiveBadFrames = 0;
            lastValidFrameMs     = millis();
            modeOfflinePublished = false;
            buildFrameHex();
            updateTempsFromFrame();
            mqttPublish();
            scanOnFrame();   // command scanner sends in the gap after a frame
          } else {
            scanBadFrames++; // garbled on the wire - ignore it completely
            if (consecutiveBadFrames < 250) consecutiveBadFrames++;
          }
        }
        break;
    }
  }
}

// ------------------------------------------------------------
// Manual Home Assistant discovery handler
// (used from MQTT config page)
// ------------------------------------------------------------
void handleHADiscovery() {
  publishDiscovery();

  String html;
  html += "<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><title>HA Discovery</title>";
  html += "<style>body{font-family:Arial;background:#f4f4f4;padding:30px;} .box{max-width:700px;margin:auto;background:white;padding:30px;border-radius:10px;} .good{color:green;} .bad{color:red;} .btn{display:inline-block;padding:10px 16px;background:#0078d7;color:white;text-decoration:none;border-radius:6px;margin-top:15px;}</style>";
  html += "</head><body><div class=\"box\"><h1>Home Assistant Discovery</h1>";

  if (discoveryPublished) {
    html += "<h2 class=\"good\">SUCCESS</h2>";
  } else {
    html += "<h2 class=\"bad\">FAILED</h2>";
  }

  html += "<p>" + discoveryStatus + "</p>";
  html += "<p>Successful publishes: " + String(discoverySuccessCount) + "</p>";
  html += "<p>Failed attempts: " + String(discoveryFailCount) + "</p>";
  html += "<p><a class=\"btn\" href=\"/mqtt\">Back to MQTT Config</a></p>";
  html += "</div></body></html>";

  server.send(200, "text/html", html);
}

// ------------------------------------------------------------
// Main web UI page
// ------------------------------------------------------------
void handleRoot() {
  String html;

  html += "<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><title>SunSwitch Controller</title>";
  html += "<style>";
  html += "body{font-family:Arial;background:#f4f4f4;margin:0;padding:0;}";
  html += ".container{width:90%;max-width:900px;margin:20px auto;background:#fff;padding:20px 30px;border-radius:10px;box-shadow:0 0 10px rgba(0,0,0,0.15);}";
  html += "h1{text-align:center;margin-bottom:10px;}";
  html += "h2{margin-top:25px;border-bottom:2px solid #ddd;padding-bottom:5px;color:#444;}";
  html += ".section{margin-top:10px;padding:10px 15px;background:#fafafa;border:1px solid #ddd;border-radius:8px;}";
  html += "table{width:100%;border-collapse:collapse;margin-top:10px;}";
  html += "th,td{padding:8px;border:1px solid #ccc;text-align:center;}";
  html += ".btn{display:inline-block;padding:10px 16px;margin:8px 0;background:#0078d7;color:white;text-decoration:none;border-radius:6px;font-weight:bold;}";
  html += ".btn:hover{background:#005ea6;}";
  html += "input[type=text],select{padding:8px;width:200px;border-radius:6px;border:1px solid #aaa;margin-top:5px;}";
  html += "input[type=submit]{padding:8px 14px;border-radius:6px;background:#0078d7;color:white;border:none;cursor:pointer;margin-left:10px;}";
  html += "input[type=submit]:hover{background:#005ea6;}";
  html += "pre{background:#eee;padding:10px;border-radius:6px;overflow-x:auto;}";
  html += "</style></head><body><div class=\"container\">";

  html += "<h1>SunSwitch Controller</h1>";

  // WiFi status
  html += "<h2>WiFi Status</h2><div class=\"section\">";
  html += "<b>SSID:</b> " + String(WiFi.SSID()) + "<br>";
  html += "<b>IP Address:</b> " + WiFi.localIP().toString() + "<br>";
  html += "<b>Signal Strength:</b> " + String(WiFi.RSSI()) + " dBm (" + wifiQuality(WiFi.RSSI()) + ")<br>";
  unsigned long upS = millis() / 1000;
  html += "<b>Wemos uptime:</b> " + String(upS / 86400) + "d " + String((upS / 3600) % 24) + "h " + String((upS / 60) % 60) + "m";
  html += " &nbsp; <b>Last restart reason:</b> " + ESP.getResetReason() + "<br>";
  html += "<b>WiFi drops since restart:</b> " + String(wifiDropCount);
  html += " &nbsp; <b>Free memory:</b> " + String(ESP.getFreeHeap()) + " bytes";
  html += "</div>";

  // Controller status & sensors
  html += "<h2>Controller Status & Sensors</h2><div class=\"section\">";
  if (haveConfirmed) {
    html += "<b>Controller Mode:</b> " + controllerModeStr + "<br>";
    html += "<b>Set Temperature:</b> " + String(setTempC, 0) + " °C<br>";
    html += "<b>Pool Sensor 1:</b> " + String(poolTemp1C, 0) + " °C<br>";
    html += "<b>Pool Sensor 2:</b> " + String(poolTemp2C, 0) + " °C<br>";
    html += "<b>Roof Temperature:</b> " + String(roofTempC, 0) + " °C<br>";
    html += "<b>Pump Status:</b> " + String(pumpStatus == 1 ? "ON" : "OFF") + "<br>";
  } else {
    html += "<i>No confirmed values yet (needs two good frames in a row).</i>";
  }
  html += "<br><small>Bad frames rejected: " + String(scanBadFrames) +
          " &nbsp; Frames with a value held for a 2nd look: " + String(glitchesFiltered) + "</small>";
  html += "</div>";

  // Pool sensor selection
  html += "<h2>Select Pool Sensor for Home Assistant</h2><div class=\"section\">";
  html += "<form method=\"POST\" action=\"/poolsel\">";
  html += "<select name=\"poolsel\">";
  html += "<option value=\"1\"" + String(selectedPoolSensor == 1 ? " selected" : "") + ">Pool Sensor 1</option>";
  html += "<option value=\"2\"" + String(selectedPoolSensor == 2 ? " selected" : "") + ">Pool Sensor 2</option>";
  html += "</select>";
  html += "<input type=\"submit\" value=\"Save\">";
  html += "</form></div>";

  // Command sender
  html += "<h2>Send Command</h2><div class=\"section\">";
  html += "<form method=\"POST\" action=\"/sendcmd\">";
  html += "<input type=\"text\" name=\"cmd\" placeholder=\"Command (supports \\\\r and \\\\n)\">";
  html += "<input type=\"submit\" value=\"Send\">";
  html += "</form></div>";

  // Tools
  html += "<h2>Tools</h2><div class=\"section\">";
  html += "<a class=\"btn\" style=\"margin-right:10px;\" href=\"/mqtt\">MQTT Configuration</a>";
  html += "<a class=\"btn\" style=\"margin-right:10px;\" href=\"/update\">Firmware Update</a>";
  html += "<a class=\"btn\" style=\"margin-right:10px;\" href=\"/wifi\">WiFi Settings</a>";
  html += "<a class=\"btn\" style=\"margin-right:10px;\" href=\"/scanner\">Command Scanner</a>";
  html += "</div>";

  // Raw frame
  html += "<h2>Raw Frame</h2><div class=\"section\"><pre>";
  html += lastFrameHex;
  html += "</pre></div>";

  // Debug bytes
  html += "<h2>Debug Bytes</h2><div class=\"section\">";
  html += "<table><tr><th>Byte</th><th>Hex</th><th>Decimal</th><th>Function</th></tr>";
  int debugBytes[] = {7,8,9,14,15,16,17,18,19};
  for (int i = 0; i < 9; i++) {
    int idx = debugBytes[i];
    uint8_t val = frameValid ? frame[idx] : 0;

    String func = "";
    if (idx == 14) {
    if      (frame[idx] == 0x08) func = "Controller Mode (Controller Off)";   
    else if (frame[idx] == 0x09) func = "Controller Mode (Automatic)";
    else if (frame[idx] == 0x11) func = "Controller Mode (Manual)";
    else if (frame[idx] == 0x03) func = "Controller Mode (Winter)";
    else if (frame[idx] == 0x49) func = "Controller Mode (Set Temp)";
    else if (frame[idx] == 0x91) func = "Controller Mode (Continuous Manual)";
    else func = "Controller Mode (Unknown)";
    }

    else if (idx == 15) func = "Pump Status";
    else if (idx == 8)  func = (frame[idx] > 0x00) ? "Energy Saving Timer (active)" : "Energy Saving Timer (idle)";

    html += "<tr><td>" + String(idx) + "</td><td>" + toHex(val) +
            "</td><td>" + String(val) + "</td><td>" + func + "</td></tr>";
  }
  html += "</table></div>";

  // Runtime
  html += "<h2>System Runtime</h2><div class=\"section\">";
  html += getRuntimeString();
  html += "</div>";

  html += "</div></body></html>";

  server.send(200, "text/html", html);
}

// ------------------------------------------------------------
// Pool sensor selection handler
// ------------------------------------------------------------
void handlePoolSel() {
  if (server.method() == HTTP_POST && server.hasArg("poolsel")) {
    selectedPoolSensor = server.arg("poolsel").toInt();
    if (selectedPoolSensor != 1 && selectedPoolSensor != 2)
      selectedPoolSensor = 1;

    savePoolSelection();
  }

  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "");
}

// ------------------------------------------------------------
// Command sender handler
// ------------------------------------------------------------
void handleSendCmd() {
  if (server.method() == HTTP_POST && server.hasArg("cmd")) {
    String c = server.arg("cmd");

    // Support \r and \n escapes
    c.replace("\\r", "\r");
    c.replace("\\n", "\n");

    for (unsigned int i = 0; i < c.length(); i++) {
      sunSerial.write(c[i]);
    }
  }

  String html;
  html += "<!DOCTYPE html><html><body>";
  html += "<h1>Command Sent</h1>";
  html += "<p>Sent (escaped): " + server.arg("cmd") + "</p>";
  html += "<p><a href=\"/\"><button>Back</button></a></p>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

// ------------------------------------------------------------
// MQTT configuration page (with username/password)
// Includes MQTT connection status + HA discovery controls
// ------------------------------------------------------------
void handleMqttConfig() {
  // Handle POST to save settings
  if (server.method() == HTTP_POST) {
    if (server.hasArg("host"))        mqttHost       = server.arg("host");
    if (server.hasArg("port"))        mqttPort       = server.arg("port").toInt();
    if (server.hasArg("user"))        mqttUser       = server.arg("user");
    if (server.hasArg("pass"))        mqttPass       = server.arg("pass");
    if (server.hasArg("topic_set"))   mqttTopicSet   = server.arg("topic_set");
    if (server.hasArg("topic_pool"))  mqttTopicPool  = server.arg("topic_pool");
    if (server.hasArg("topic_roof"))  mqttTopicRoof  = server.arg("topic_roof");
    if (server.hasArg("topic_pump"))  mqttTopicPump  = server.arg("topic_pump");
    if (server.hasArg("topic_error")) mqttTopicError = server.arg("topic_error");
    if (server.hasArg("topic_mode"))  mqttTopicMode  = server.arg("topic_mode");

    saveMqttConfig();

    // Force an immediate MQTT reconnect using the new settings.
    mqtt.disconnect();
    mqttWill.disconnect();
    lastMqttAttempt = millis() - 10000UL;
    lastWillAttempt = millis() - 10000UL;
  }

  String html;
  html += "<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><title>MQTT Configuration</title>";
  html += "<style>";
  html += "body{font-family:Arial;background:#f4f4f4;margin:0;padding:0;}";
  html += ".container{width:90%;max-width:900px;margin:20px auto;background:#fff;padding:20px 30px;border-radius:10px;box-shadow:0 0 10px rgba(0,0,0,0.15);}";
  html += "h1{text-align:center;margin-bottom:10px;}";
  html += ".section{margin-top:10px;padding:10px 15px;background:#fafafa;border:1px solid #ddd;border-radius:8px;}";
  html += "input[type=text]{padding:8px;width:300px;border-radius:6px;border:1px solid #aaa;margin-top:5px;}";
  html += "input[type=password]{padding:8px;width:300px;border-radius:6px;border:1px solid #aaa;margin-top:5px;}";
  html += "input[type=submit]{padding:10px 16px;border-radius:6px;background:#0078d7;color:white;border:none;cursor:pointer;margin-top:10px;}";
  html += "input[type=submit]:hover{background:#005ea6;}";
  html += ".btn{display:inline-block;padding:10px 16px;background:#0078d7;color:white;text-decoration:none;border-radius:6px;font-weight:bold;margin-top:15px;}";
  html += ".btn:hover{background:#005ea6;}";
  html += "</style></head><body><div class=\"container\">";

  html += "<h1>MQTT Configuration</h1>";

  // MQTT connection + discovery status at top
  html += "<div class=\"section\">";
  html += "<b>MQTT Connection:</b> ";
  html += mqtt.connected()
    ? "<span style=\"color:green;font-weight:bold\">CONNECTED</span><br>"
    : "<span style=\"color:red;font-weight:bold\">NOT CONNECTED</span><br>";
  html += "<b>Discovery Status:</b> " + discoveryStatus + "<br>";
  html += "<b>Discovery successful:</b> " + String(discoverySuccessCount) + "<br>";
  html += "<b>Discovery failed:</b> " + String(discoveryFailCount) + "<br>";
  html += "<a class=\"btn\" href=\"/ha_discovery\">Publish HA Discovery Now</a>";
  html += "</div>";

  html += "<form method=\"POST\">";

  html += "<div class=\"section\"><b>MQTT Host:</b><br>";
  html += "<input type=\"text\" name=\"host\" value=\"" + mqttHost + "\"></div>";

  html += "<div class=\"section\"><b>MQTT Port:</b><br>";
  html += "<input type=\"text\" name=\"port\" value=\"" + String(mqttPort) + "\"></div>";

  html += "<div class=\"section\"><b>MQTT Username:</b><br>";
  html += "<input type=\"text\" name=\"user\" value=\"" + mqttUser + "\"></div>";

  html += "<div class=\"section\"><b>MQTT Password:</b><br>";
  html += "<input type=\"password\" name=\"pass\" value=\"" + mqttPass + "\"></div>";

  html += "<div class=\"section\"><b>Set Temperature Topic:</b><br>";
  html += "<input type=\"text\" name=\"topic_set\" value=\"" + mqttTopicSet + "\"></div>";

  html += "<div class=\"section\"><b>Pool Temperature Topic:</b><br>";
  html += "<input type=\"text\" name=\"topic_pool\" value=\"" + mqttTopicPool + "\"></div>";

  html += "<div class=\"section\"><b>Roof Temperature Topic:</b><br>";
  html += "<input type=\"text\" name=\"topic_roof\" value=\"" + mqttTopicRoof + "\"></div>";

  html += "<div class=\"section\"><b>Pump Status Topic:</b><br>";
  html += "<input type=\"text\" name=\"topic_pump\" value=\"" + mqttTopicPump + "\"></div>";

  html += "<div class=\"section\"><b>Error Topic (reserved):</b><br>";
  html += "<input type=\"text\" name=\"topic_error\" value=\"" + mqttTopicError + "\"></div>";

  html += "<div class=\"section\"><b>Controller Mode Topic:</b><br>";
  html += "<input type=\"text\" name=\"topic_mode\" value=\"" + mqttTopicMode + "\"></div>";

  html += "<input type=\"submit\" value=\"Save & Reconnect MQTT\">";
  html += "</form>";

  html += "<div class=\"section\">";
  html += "<a class=\"btn\" href=\"/\">Back to Main Page</a>";
  html += "</div>";

  html += "</div></body></html>";

  server.send(200, "text/html", html);
}

// ------------------------------------------------------------
// WiFi settings (WiFiManager portal trigger)
// ------------------------------------------------------------
void handleWifiSettings() {
  String html;
  html += "<!DOCTYPE html><html><head><meta charset=\"UTF-8\"><title>WiFi Settings</title></head><body>";
  html += "<h1>WiFi Settings</h1>";
  html += "<p>This will start the WiFiManager portal. Connect to the AP and configure WiFi.</p>";
  html += "<p><a href=\"/start_portal\"><button>Start WiFi Portal</button></a></p>";
  html += "<p><a href=\"/\"><button>Back</button></a></p>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

// Start WiFiManager portal
void handleStartPortal() {
  server.send(200, "text/plain", "Starting WiFiManager portal...");
  WiFiManager wm;
  wm.setConfigPortalTimeout(180);   // give up after 3 minutes so the device never stays stuck here
  wm.startConfigPortal("SunSwitchConfig");
}


// ============================================================
// COMMAND SCANNER
// ------------------------------------------------------------
// Sends one candidate command at a time, waits, then compares
// set temp (byte 10), mode (byte 14) and pump (byte 15) with
// the values seen just before the command went out.
//
//   Phase 1 : single byte 00..FF
//   Phase 2 : "Sun" frames carrying a new set temp / mode
//   Phase 3 : 'S' 'u' 'n' + byte 00..FF
//   Phase 4 : byte 00..FF + CR
//
// Commands are sent immediately after a status frame has been
// received, so they land in the quiet gap between frames.
// ============================================================
#define SCAN_MAX_PAYLOAD 26
#define SCAN_MAX_HITS    24
#define SCAN_LOG_LINES   14
#define SCAN_PHASES      5

enum ScanState { SCAN_IDLE, SCAN_WAIT_GAP, SCAN_WAIT_RESULT, SCAN_PAUSED, SCAN_DONE, SCAN_WAIT_ARM, SCAN_SETTLE };
ScanState scanState = SCAN_IDLE;

uint8_t       scanPhase       = 1;
int           scanIndex       = 0;
unsigned long scanWaitMs      = 5000;
bool          scanStopOnHit   = false;
bool          scanAllPhases   = false;   // keep going to later phases even after a hit
bool          scanOneShot     = false;   // manual send / resend of a single command
int           scanRunHits     = 0;
uint32_t      scanTried       = 0;

uint8_t       scanPayload[SCAN_MAX_PAYLOAD];
uint8_t       scanPayloadLen  = 0;
String        scanPayloadNote = "";
unsigned long scanSentAt      = 0;
unsigned long scanLastFrameAt = 0;
uint8_t       scanFramesSince = 0;
bool          scanStable      = false;
uint8_t       scanPrevSet = 0, scanPrevMode = 0;
uint8_t       scanBaseSet = 0, scanBaseMode = 0, scanBasePump = 0;
String        scanMessage     = "Idle. Press Start.";

// Optional "send first" command (e.g. 52 to enter Set Temp Mode).
// Before each candidate, if the controller is not already in the
// mode that command produces, it is sent and we wait for the mode
// to change before sending the candidate.
#define SCAN_MAX_PREFIX 8
uint8_t       scanPrefix[SCAN_MAX_PREFIX];
uint8_t       scanPrefixLen   = 0;
bool          scanArmedKnown  = false;   // have we seen what mode the prefix produces?
uint8_t       scanArmedMode   = 0;       // mode byte after the prefix took effect
uint8_t       scanIdleMode    = 0;       // mode byte before the prefix was sent
bool          scanPrefixUsed  = false;   // this step ran with the prefix active
unsigned long scanArmSentAt   = 0;

// Reply capture: any bytes from the controller that are NOT part
// of a "Sun" status frame (banner text, error replies, echoes).
#define SCAN_REPLY_MAX 48
#define SCAN_STRAY_MAX 120
uint8_t       scanReply[SCAN_REPLY_MAX];   // bytes seen since the last command
uint8_t       scanReplyLen    = 0;
uint8_t       scanStray[SCAN_STRAY_MAX];   // rolling history, shown on the page
uint8_t       scanStrayLen    = 0;
uint32_t      scanBaseRuntime = 0;         // controller runtime when the command went out
unsigned long scanSettleAt    = 0;
uint32_t      scanPrevRuntime = 0;         // runtime in the previous frame
uint8_t       scanPrevPump    = 0;
uint32_t      scanBaseBad     = 0;         // rejected-frame count when the command went out

struct ScanHit {
  uint8_t data[SCAN_MAX_PAYLOAD];
  uint8_t len;
  char    desc[80];
};
ScanHit scanHits[SCAN_MAX_HITS];
uint8_t scanHitCount = 0;

String scanLog[SCAN_LOG_LINES];
int    scanLogCount = 0;

// ---------- small helpers ----------
String scanHex(const uint8_t *d, uint8_t len) {
  String s;
  for (uint8_t i = 0; i < len; i++) {
    if (i) s += " ";
    s += toHex(d[i]);
  }
  return s;
}

// Printable text version of a byte buffer (non-printables shown as '.')
String scanAscii(const uint8_t *d, uint8_t len) {
  String s;
  for (uint8_t i = 0; i < len; i++) s += (d[i] >= 0x20 && d[i] <= 0x7E) ? (char)d[i] : '.';
  return s;
}

// Called by the frame parser for every byte that is not part of a frame.
void scanStrayByte(uint8_t b) {
  if (scanReplyLen < SCAN_REPLY_MAX) scanReply[scanReplyLen++] = b;
  if (scanStrayLen >= SCAN_STRAY_MAX) {
    memmove(scanStray, scanStray + 1, SCAN_STRAY_MAX - 1);
    scanStrayLen = SCAN_STRAY_MAX - 1;
  }
  scanStray[scanStrayLen++] = b;
}

uint32_t scanRuntime() {
  return ((uint32_t)frame[3] << 16) | ((uint32_t)frame[4] << 8) | frame[5];
}

String scanModeName(uint8_t m) {
  switch (m) {
    case 0x08: return "Controller Off";
    case 0x09: return "Automatic";
    case 0x11: return "Manual";
    case 0x03: return "Winter Mode";
    case 0x49: return "Set Temp Mode";
    case 0x91: return "Continuous Manual";
  }
  return "0x" + toHex(m);
}

void scanAddLog(const String &line) {
  if (scanLogCount < SCAN_LOG_LINES) {
    scanLog[scanLogCount++] = line;
  } else {
    for (int i = 1; i < SCAN_LOG_LINES; i++) scanLog[i - 1] = scanLog[i];
    scanLog[SCAN_LOG_LINES - 1] = line;
  }
}

int scanPhaseCount(uint8_t phase) {
  if (phase == 2) return 7;
  if (phase == 5) return 208;      // 52 letters x 4 forms
  return 256;
}

const char *scanPhaseName(uint8_t phase) {
  switch (phase) {
    case 1: return "1: single byte";
    case 2: return "2: Sun frame, direct set";
    case 3: return "3: Sun + byte";
    case 4: return "4: byte + CR";
    case 5: return "5: letter + set temp";
  }
  return "?";
}

// ---------- checksum guessing ----------
// Types: 1/2 sum8, 3/4 xor, 5/6 sum16 big-endian, 7/8 sum16
// little-endian, 9/10 negated sum8. Odd types cover the frame
// from byte 0, even types from byte 3 (after "Sun").
void scanApplyChecksum(uint8_t *f, uint8_t type) {
  if (type == 0) return;
  uint8_t start = (type & 1) ? 0 : 3;
  uint8_t kind  = (type - 1) / 2;
  uint8_t last  = (kind == 2 || kind == 3) ? 20 : 21;
  uint16_t s = 0;
  uint8_t  x = 0;
  for (uint8_t i = start; i <= last; i++) { s += f[i]; x ^= f[i]; }
  switch (kind) {
    case 0: f[22] = s & 0xFF; break;
    case 1: f[22] = x; break;
    case 2: f[21] = s >> 8;   f[22] = s & 0xFF; break;
    case 3: f[21] = s & 0xFF; f[22] = s >> 8;   break;
    case 4: f[22] = (uint8_t)(0 - (s & 0xFF)); break;
  }
}

uint8_t scanDetectChecksum() {
  if (!frameValid) return 0;
  uint8_t tmp[23];
  for (uint8_t t = 1; t <= 10; t++) {
    memcpy(tmp, frame, 23);
    scanApplyChecksum(tmp, t);
    if (memcmp(tmp, frame, 23) == 0) return t;
  }
  return 0;
}

String scanChecksumName(uint8_t type) {
  if (type == 0) return "none recognised";
  const char *kinds[] = { "sum8", "xor", "sum16 BE", "sum16 LE", "negated sum8" };
  String s = kinds[(type - 1) / 2];
  s += (type & 1) ? " from byte 0" : " from byte 3";
  return s;
}

// ---------- build the candidate command ----------
// Returns false if this index should be skipped.
bool scanBuildPayload() {
  uint8_t b = (uint8_t)scanIndex;

  switch (scanPhase) {
    case 1:
      scanPayload[0] = b;
      scanPayloadLen = 1;
      scanPayloadNote = "single byte";
      return true;

    case 3:
      scanPayload[0] = 'S'; scanPayload[1] = 'u'; scanPayload[2] = 'n';
      scanPayload[3] = b;
      scanPayloadLen = 4;
      scanPayloadNote = "Sun + byte";
      return true;

    case 4:
      scanPayload[0] = b;
      scanPayload[1] = 0x0D;
      scanPayloadLen = 2;
      scanPayloadNote = "byte + CR";
      return true;

    case 5: {
      // Letter (A-Z, a-z) followed by a new set temp, in four forms.
      uint8_t li      = scanIndex / 4;
      char    letter  = (li < 26) ? ('A' + li) : ('a' + li - 26);
      if (letter == 'R') return false;           // R restarts the controller - skip
      uint8_t t = (frame[10] >= 35) ? frame[10] - 1 : frame[10] + 1;
      char    d1 = '0' + (t / 10) % 10, d2 = '0' + t % 10;
      String  L  = String(letter);
      scanPayload[0] = letter;
      switch (scanIndex % 4) {
        case 0:
          scanPayload[1] = t; scanPayloadLen = 2;
          scanPayloadNote = L + " + temp " + String(t) + " as one byte";
          break;
        case 1:
          scanPayload[1] = t; scanPayload[2] = 0x0D; scanPayloadLen = 3;
          scanPayloadNote = L + " + temp " + String(t) + " as one byte + CR";
          break;
        case 2:
          scanPayload[1] = d1; scanPayload[2] = d2; scanPayload[3] = 0x0D; scanPayloadLen = 4;
          scanPayloadNote = "text " + L + String(t) + " + CR";
          break;
        case 3:
          scanPayload[1] = ' '; scanPayload[2] = d1; scanPayload[3] = d2; scanPayload[4] = 0x0D; scanPayloadLen = 5;
          scanPayloadNote = "text " + L + " " + String(t) + " + CR";
          break;
      }
      return true;
    }
  }

  // Phase 2: frames modelled on what the controller sends us
  uint8_t cs     = scanDetectChecksum();
  uint8_t cur    = frame[10];
  uint8_t target = (cur >= 35) ? cur - 1 : cur + 1;
  uint8_t mode   = (frame[14] == 0x11) ? 0x09 : 0x11;

  memcpy(scanPayload, frame, 23);
  scanPayloadLen = 23;

  switch (scanIndex) {
    case 0:
      scanPayload[10] = target;
      scanPayloadNote = "full frame, set temp " + String(target);
      return true;
    case 1:
      if (cs == 0) return false;
      scanPayload[10] = target;
      scanApplyChecksum(scanPayload, cs);
      scanPayloadNote = "full frame, set temp " + String(target) + ", checksum fixed";
      return true;
    case 2:
      scanPayload[3] = target;
      scanPayloadLen = 4;
      scanPayloadNote = "Sun + set temp " + String(target);
      return true;
    case 3:
      scanPayload[0] = 'S'; scanPayload[1] = 'U'; scanPayload[2] = 'N';
      scanPayload[3] = target;
      scanPayloadLen = 4;
      scanPayloadNote = "SUN (caps) + set temp " + String(target);
      return true;
    case 4:
      scanPayload[10] = target;
      scanPayloadLen = 11;
      scanPayloadNote = "frame cut after set temp " + String(target);
      return true;
    case 5:
      scanPayload[14] = mode;
      scanPayloadNote = "full frame, mode " + scanModeName(mode);
      return true;
    case 6:
      if (cs == 0) return false;
      scanPayload[14] = mode;
      scanApplyChecksum(scanPayload, cs);
      scanPayloadNote = "full frame, mode " + scanModeName(mode) + ", checksum fixed";
      return true;
  }
  return false;
}

// ---------- hit storage (SPIFFS: /hits.cfg) ----------
void saveHits() {
  File f = SPIFFS.open("/hits.cfg", "w");
  if (!f) return;
  for (uint8_t i = 0; i < scanHitCount; i++) {
    String line;
    for (uint8_t j = 0; j < scanHits[i].len; j++) line += toHex(scanHits[i].data[j]);
    line += "|";
    line += scanHits[i].desc;
    f.println(line);
  }
  f.close();
}

uint8_t scanParseHex(const String &in, uint8_t *out, uint8_t maxLen) {
  uint8_t n = 0;
  int     hi = -1;
  for (unsigned int i = 0; i < in.length(); i++) {
    char c = in[i];
    int  v;
    if      (c >= '0' && c <= '9') v = c - '0';
    else if (c >= 'a' && c <= 'f') v = c - 'a' + 10;
    else if (c >= 'A' && c <= 'F') v = c - 'A' + 10;
    else continue;
    if (hi < 0) hi = v;
    else {
      if (n < maxLen) out[n++] = (uint8_t)((hi << 4) | v);
      hi = -1;
    }
  }
  return n;
}

void loadHits() {
  scanHitCount = 0;
  if (!SPIFFS.exists("/hits.cfg")) return;
  File f = SPIFFS.open("/hits.cfg", "r");
  if (!f) return;
  while (f.available() && scanHitCount < SCAN_MAX_HITS) {
    String line = f.readStringUntil('\n');
    line.trim();
    int bar = line.indexOf('|');
    if (bar <= 0) continue;
    ScanHit &h = scanHits[scanHitCount];
    h.len = scanParseHex(line.substring(0, bar), h.data, SCAN_MAX_PAYLOAD);
    if (h.len == 0) continue;
    strncpy(h.desc, line.substring(bar + 1).c_str(), sizeof(h.desc) - 1);
    h.desc[sizeof(h.desc) - 1] = 0;
    scanHitCount++;
  }
  f.close();
}

void scanAddHit(const String &desc) {
  // Same command already listed? Just refresh its description.
  uint8_t slot = scanHitCount;
  for (uint8_t i = 0; i < scanHitCount; i++) {
    if (scanHits[i].len == scanPayloadLen &&
        memcmp(scanHits[i].data, scanPayload, scanPayloadLen) == 0) {
      slot = i;
      break;
    }
  }
  if (slot >= SCAN_MAX_HITS) return;
  memcpy(scanHits[slot].data, scanPayload, scanPayloadLen);
  scanHits[slot].len = scanPayloadLen;
  strncpy(scanHits[slot].desc, desc.c_str(), sizeof(scanHits[slot].desc) - 1);
  scanHits[slot].desc[sizeof(scanHits[slot].desc) - 1] = 0;
  if (slot == scanHitCount) scanHitCount++;
  saveHits();
}

// ---------- state machine ----------
// Move to the next candidate (or finish).
void scanAdvance() {
  scanIndex++;
  while (true) {
    if (scanIndex >= scanPhaseCount(scanPhase)) {
      if (scanPhase >= SCAN_PHASES) {
        scanState = SCAN_DONE;
        scanMessage = "Finished all phases. Hits this run: " + String(scanRunHits);
        return;
      }
      if (scanRunHits > 0 && !scanAllPhases) {
        scanState = SCAN_DONE;
        scanMessage = "Phase " + String(scanPhase) + " finished with " + String(scanRunHits) +
                      " hit(s), so later phases were not run.";
        return;
      }
      scanAddLog("--- Phase " + String(scanPhase) + " finished, nothing found. Moving on. ---");
      scanPhase++;
      scanIndex = 0;
    }
    if (scanBuildPayload()) break;
    scanIndex++;
  }
  scanState = SCAN_WAIT_GAP;
  scanLastFrameAt = millis();
  scanMessage = "Waiting for a frame gap to send...";
}

// Send the candidate command and start the result timer.
void scanSendCandidate() {
  scanBaseSet  = frame[10];
  scanBaseMode = frame[14];
  scanBasePump = frame[15];
  scanBaseRuntime = scanRuntime();
  scanReplyLen    = 0;
  scanBaseBad     = scanBadFrames;

  for (uint8_t i = 0; i < scanPayloadLen; i++) sunSerial.write(scanPayload[i]);

  scanSentAt      = millis();
  scanFramesSince = 0;
  scanTried++;
  scanState   = SCAN_WAIT_RESULT;
  scanMessage = "Sent. Watching for a change...";
}

// Called every time a complete status frame has been received.
void scanOnFrame() {
  // "Stable" = this frame agrees with the one before it: same set temp,
  // mode and pump, and a runtime that has only ticked forward a little.
  // A single frame damaged in transit can therefore never cause a hit.
  uint32_t rtNow = scanRuntime();
  scanStable   = (frame[10] == scanPrevSet && frame[14] == scanPrevMode && frame[15] == scanPrevPump &&
                  rtNow >= scanPrevRuntime && rtNow - scanPrevRuntime <= 30);
  scanPrevSet  = frame[10];
  scanPrevMode = frame[14];
  scanPrevPump = frame[15];
  scanPrevRuntime = rtNow;
  scanLastFrameAt = millis();
  if (scanFramesSince < 250) scanFramesSince++;

  if (scanState == SCAN_WAIT_ARM) {
    // Prefix has been sent: wait for two matching frames showing a new mode.
    if (scanFramesSince < 2 || !scanStable) return;
    if (frame[14] != scanIdleMode) {
      scanArmedMode  = frame[14];
      scanArmedKnown = true;
      scanSendCandidate();
    } else if (millis() - scanArmSentAt > 4000UL) {
      scanAddLog("!!   'Send first' " + scanHex(scanPrefix, scanPrefixLen) +
                 " did not change the mode - sending the command anyway.");
      scanSendCandidate();
    }
    return;
  }

  if (scanState == SCAN_SETTLE) {
    // After a restart: carry on once the controller is back in its old mode.
    if (scanStable && frame[14] == scanBaseMode) scanAdvance();
    return;
  }

  if (scanState != SCAN_WAIT_GAP) return;
  if (!scanStable) return;                    // want a steady baseline first

  if (!scanOneShot) scanBuildPayload();       // rebuild from the freshest frame

  scanPrefixUsed = scanPrefixLen > 0;
  if (scanPrefixUsed && !(scanArmedKnown && frame[14] == scanArmedMode)) {
    scanIdleMode = frame[14];
    for (uint8_t i = 0; i < scanPrefixLen; i++) sunSerial.write(scanPrefix[i]);
    scanArmSentAt   = millis();
    scanFramesSince = 0;
    scanState   = SCAN_WAIT_ARM;
    scanMessage = "Sent " + scanHex(scanPrefix, scanPrefixLen) + " first, waiting for the mode to change...";
    return;
  }
  scanSendCandidate();
}

// Compare the latest frame with the baseline taken before sending.
void scanEvaluate() {
  bool setCh  = frame[10] != scanBaseSet;
  bool modeCh = frame[14] != scanBaseMode;
  bool pumpCh = frame[15] != scanBasePump;
  // With a "send first" command the mode is expected to drop back to
  // where it started, and the pump may react to the prefix itself, so
  // neither of those counts as a hit on its own.
  bool modeBack = scanPrefixUsed && modeCh && frame[14] == scanIdleMode;
  bool restart  = scanRuntime() < scanBaseRuntime;   // runtime counter went backwards
  bool echo     = scanReplyLen == scanPayloadLen && memcmp(scanReply, scanPayload, scanPayloadLen) == 0;
  // Stray bytes that are really a status frame with a damaged header
  // (reception error) must not be mistaken for a reply.
  bool debris   = (scanBadFrames != scanBaseBad) ||
                  (scanReplyLen >= 18 && scanReply[scanReplyLen - 2] == 0x0A && scanReply[scanReplyLen - 1] == 0x0D);
  bool reply    = scanReplyLen >= 2 && !echo && !debris;
  bool hit      = restart || reply || setCh || (modeCh && !modeBack) || (pumpCh && !scanPrefixUsed);

  String res;
  if (restart) res += "CONTROLLER RESTARTED. ";
  if (setCh)  res += "SET TEMP " + String(scanBaseSet) + " -> " + String(frame[10]) + ". ";
  if (modeCh) res += String(modeBack ? "mode back to " : "MODE " + scanModeName(scanBaseMode) + " -> ") +
                     scanModeName(frame[14]) + ". ";
  if (pumpCh) res += "PUMP " + String(scanBasePump) + " -> " + String(frame[15]) +
                     ((setCh || modeCh) ? ". " : " (pump only - could be coincidence). ");
  if (scanReplyLen > 0) {
    res += String(echo ? "echo only: " : (debris ? "garbled status frame (ignored): " : "REPLY: ")) + scanHex(scanReply, scanReplyLen) +
           " \"" + scanAscii(scanReply, scanReplyLen) + "\". ";
  }
  if (res.length() == 0) res = "no change";
  if (!hit && debris && !setCh && !modeCh && !pumpCh && scanReplyLen > 0) res = "no change (one garbled status frame ignored)";

  String cmd = scanHex(scanPayload, scanPayloadLen);
  if (scanPrefixUsed) scanPayloadNote = "after " + scanHex(scanPrefix, scanPrefixLen) + ", " + scanPayloadNote;
  scanAddLog((hit ? "HIT  " : "     ") + cmd + "  [" + scanPayloadNote + "]  " + res);

  if (scanOneShot) {
    scanOneShot = false;
    scanState   = SCAN_IDLE;
    scanMessage = "Sent " + cmd + ": " + res;
    return;
  }

  if (hit) {
    scanRunHits++;
    scanAddHit(res + "[" + scanPayloadNote + "]");
    if (scanStopOnHit) {
      scanState   = SCAN_PAUSED;
      scanMessage = "Paused on hit: " + cmd + " - " + res + " Press Resume to continue.";
      return;
    }
  }
  if (restart) {
    scanState    = SCAN_SETTLE;
    scanSettleAt = millis();
    scanMessage  = cmd + " restarted the controller. Waiting for it to settle before continuing...";
    return;
  }
  scanAdvance();
}

// Called from loop().
void scanLoop() {
  unsigned long now = millis();

  if (scanState == SCAN_WAIT_GAP || scanState == SCAN_WAIT_ARM) {
    if (now - scanLastFrameAt > 15000UL) {
      scanState   = SCAN_PAUSED;
      scanOneShot = false;
      scanIndex--;                             // retry this one on Resume
      scanMessage = "Paused: no steady frames from the controller for 15 s.";
      scanAddLog("!!   No frames from controller for 15 s - paused.");
    }
  } else if (scanState == SCAN_SETTLE) {
    if (now - scanSettleAt > 90000UL) {
      scanAddLog("!!   Controller did not return to its previous mode within 90 s - continuing anyway.");
      scanAdvance();
    }
  } else if (scanState == SCAN_WAIT_RESULT) {
    unsigned long el = now - scanSentAt;
    if (el >= scanWaitMs && scanFramesSince >= 2 && scanStable) {
      scanEvaluate();
    } else if (el >= scanWaitMs + 15000UL) {
      String cmd = scanHex(scanPayload, scanPayloadLen);
      scanAddLog("!!   " + cmd + "  controller stopped sending steady frames after this command - paused.");
      scanState   = SCAN_PAUSED;
      scanOneShot = false;
      scanMessage = "Paused: controller went quiet after " + cmd +
                    ". That may itself be a reaction - check the controller, then Resume.";
    }
  }
}

bool scanBusy() {
  return scanState == SCAN_WAIT_GAP || scanState == SCAN_WAIT_RESULT || scanState == SCAN_WAIT_ARM || scanState == SCAN_SETTLE;
}

bool scanFramesAlive() {
  return frameValid && (millis() - scanLastFrameAt < 15000UL);
}

// ---------- web handlers ----------
String scanJsonEsc(const String &in) {
  String s = in;
  s.replace("\\", "\\\\");
  s.replace("\"", "\\\"");
  return s;
}

void handleScanStatus() {
  const char *stateNames[] = { "Idle", "Sending", "Waiting for result", "Paused", "Finished", "Sending first command", "Waiting for controller to restart" };
  long remain = 0;
  if (scanState == SCAN_WAIT_RESULT) {
    long el = (long)(millis() - scanSentAt);
    remain = ((long)scanWaitMs - el + 999) / 1000;
    if (remain < 0) remain = 0;
  }

  String j = "{";
  j += "\"state\":\"" + String(stateNames[scanState]) + "\",";
  j += "\"busy\":" + String(scanBusy() ? "true" : "false") + ",";
  j += "\"paused\":" + String(scanState == SCAN_PAUSED ? "true" : "false") + ",";
  j += "\"msg\":\"" + scanJsonEsc(scanMessage) + "\",";
  j += "\"phase\":\"" + String(scanPhaseName(scanPhase)) + "\",";
  j += "\"index\":" + String(scanIndex) + ",";
  j += "\"count\":" + String(scanPhaseCount(scanPhase)) + ",";
  j += "\"cmd\":\"" + (scanPayloadLen ? scanHex(scanPayload, scanPayloadLen) : String("-")) + "\",";
  j += "\"note\":\"" + scanJsonEsc(scanPayloadNote) + "\",";
  j += "\"pre\":\"" + scanHex(scanPrefix, scanPrefixLen) + "\",";
  j += "\"remain\":" + String(remain) + ",";
  j += "\"wait\":" + String(scanWaitMs / 1000) + ",";
  j += "\"tried\":" + String(scanTried) + ",";
  j += "\"frames\":" + String(scanFramesAlive() ? "true" : "false") + ",";
  j += "\"set\":" + String(frameValid ? frame[10] : 0) + ",";
  j += "\"mode\":\"" + (frameValid ? scanModeName(frame[14]) : String("-")) + "\",";
  j += "\"pump\":\"" + String(frameValid ? (frame[15] == 1 ? "ON" : "OFF") : "-") + "\",";
  j += "\"cs\":\"" + scanChecksumName(scanDetectChecksum()) + "\",";

  j += "\"bad\":" + String(scanBadFrames) + ",";
  j += "\"strayhex\":\"" + scanHex(scanStray, scanStrayLen) + "\",";
  j += "\"straytxt\":\"" + scanJsonEsc(scanAscii(scanStray, scanStrayLen)) + "\",";

  j += "\"hits\":[";
  for (uint8_t i = 0; i < scanHitCount; i++) {
    if (i) j += ",";
    j += "{\"hex\":\"" + scanHex(scanHits[i].data, scanHits[i].len) + "\",";
    j += "\"desc\":\"" + scanJsonEsc(String(scanHits[i].desc)) + "\"}";
  }
  j += "],\"log\":[";
  for (int i = 0; i < scanLogCount; i++) {
    if (i) j += ",";
    j += "\"" + scanJsonEsc(scanLog[i]) + "\"";
  }
  j += "]}";

  server.send(200, "application/json", j);
}

// Read the optional "send first" bytes from the request.
void scanReadPrefix() {
  if (!server.hasArg("pre")) return;
  uint8_t buf[SCAN_MAX_PREFIX];
  uint8_t n = scanParseHex(server.arg("pre"), buf, SCAN_MAX_PREFIX);
  if (n != scanPrefixLen || memcmp(buf, scanPrefix, n) != 0) scanArmedKnown = false;
  memcpy(scanPrefix, buf, n);
  scanPrefixLen = n;
}

void handleScanStart() {
  if (scanBusy()) { server.send(200, "text/plain", "Already running."); return; }

  if (!scanFramesAlive()) {
    scanMessage = "Cannot start: no status frames are arriving from the controller.";
    server.send(200, "text/plain", scanMessage);
    return;
  }

  if (server.hasArg("wait")) {
    long w = server.arg("wait").toInt();
    if (w < 2)  w = 2;
    if (w > 60) w = 60;
    scanWaitMs = (unsigned long)w * 1000UL;
  }
  scanReadPrefix();
  if (server.hasArg("stoponhit")) scanStopOnHit = server.arg("stoponhit") == "1";
  if (server.hasArg("allphases")) scanAllPhases = server.arg("allphases") == "1";

  scanOneShot = false;

  if (server.hasArg("resume") && scanState == SCAN_PAUSED) {
    scanAddLog("--- Resumed ---");
  } else {
    int p = server.arg("phase").toInt();
    if (p < 1 || p > SCAN_PHASES) p = 1;
    long from = strtol(server.arg("from").c_str(), NULL, 16);
    if (from < 0 || from >= scanPhaseCount(p)) from = 0;
    scanPhase   = (uint8_t)p;
    scanIndex   = (int)from - 1;
    scanRunHits = 0;
    scanAddLog("--- Started phase " + String(p) + " from " + toHex((uint8_t)from) + " ---");
  }

  scanAdvance();
  server.send(200, "text/plain", "OK");
}

void handleScanStop() {
  if (scanBusy()) {
    if (!scanOneShot && scanState != SCAN_SETTLE) scanIndex--;   // Resume retries the command in progress
    scanOneShot = false;
    scanState   = SCAN_PAUSED;
    scanMessage = "Stopped. Press Resume to carry on from the same command.";
    scanAddLog("--- Stopped by user ---");
  }
  server.send(200, "text/plain", "OK");
}

// Queue a single command; it is checked for an effect just like a scan step.
void scanQueueOneShot(const uint8_t *data, uint8_t len, const String &note) {
  memcpy(scanPayload, data, len);
  scanPayloadLen  = len;
  scanPayloadNote = note;
  scanOneShot     = true;
  scanState       = SCAN_WAIT_GAP;
  scanLastFrameAt = millis();
  scanMessage     = "Waiting for a frame gap to send...";
}

void handleScanResend() {
  int i = server.arg("i").toInt();
  if (scanBusy())                  { server.send(200, "text/plain", "Stop the scan first."); return; }
  if (i < 0 || i >= scanHitCount)  { server.send(200, "text/plain", "No such hit.");         return; }
  if (!scanFramesAlive())          { scanMessage = "Cannot send: no frames from controller.";
                                     server.send(200, "text/plain", scanMessage);            return; }
  scanReadPrefix();
  scanQueueOneShot(scanHits[i].data, scanHits[i].len, "resend");
  server.send(200, "text/plain", "OK");
}

void handleScanSendHex() {
  uint8_t buf[SCAN_MAX_PAYLOAD];
  uint8_t n = scanParseHex(server.arg("hex"), buf, SCAN_MAX_PAYLOAD);
  if (scanBusy())         { server.send(200, "text/plain", "Stop the scan first.");    return; }
  if (n == 0)             { scanMessage = "No hex bytes given.";
                            server.send(200, "text/plain", scanMessage);               return; }
  if (!scanFramesAlive()) { scanMessage = "Cannot send: no frames from controller.";
                            server.send(200, "text/plain", scanMessage);               return; }
  scanReadPrefix();
  scanQueueOneShot(buf, n, "manual");
  server.send(200, "text/plain", "OK");
}

void handleScanClear() {
  if (server.hasArg("stray")) { scanStrayLen = 0; server.send(200, "text/plain", "OK"); return; }
  scanHitCount = 0;
  saveHits();
  server.send(200, "text/plain", "OK");
}

// ---------- scanner web page ----------
const char SCAN_PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>SunSwitch Command Scanner</title>
<style>
body{font-family:Arial;background:#f4f4f4;margin:0;padding:0;}
.container{width:90%;max-width:900px;margin:20px auto;background:#fff;padding:20px 30px;border-radius:10px;box-shadow:0 0 10px rgba(0,0,0,0.15);}
h1{text-align:center;margin-bottom:10px;}
h2{margin-top:25px;border-bottom:2px solid #ddd;padding-bottom:5px;color:#444;}
.section{margin-top:10px;padding:10px 15px;background:#fafafa;border:1px solid #ddd;border-radius:8px;}
table{width:100%;border-collapse:collapse;margin-top:10px;}
th,td{padding:8px;border:1px solid #ccc;text-align:left;}
button,.btn{display:inline-block;padding:10px 16px;margin:6px 6px 6px 0;background:#0078d7;color:#fff;text-decoration:none;border-radius:6px;font-weight:bold;border:none;cursor:pointer;font-size:14px;}
button:hover,.btn:hover{background:#005ea6;}
button.go{background:#1a8f3c;} button.stop{background:#c62828;} button.grey{background:#777;}
button:disabled{background:#bbb;cursor:default;}
input[type=text],input[type=number],select{padding:8px;border-radius:6px;border:1px solid #aaa;margin:4px 10px 4px 0;}
#cmd{font-family:monospace;font-size:26px;font-weight:bold;color:#0078d7;word-break:break-all;}
#log{background:#eee;padding:10px;border-radius:6px;overflow-x:auto;font-size:12px;min-height:60px;}
.mono{font-family:monospace;}
.warn{color:#c62828;font-weight:bold;}
label{margin-right:14px;}
</style></head><body><div class="container">
<h1>SunSwitch Command Scanner</h1>

<h2>Now Trying</h2><div class="section">
<div id="cmd">-</div>
<div id="note"></div>
<p><b>State:</b> <span id="state">-</span> &nbsp; <b>Result check in:</b> <span id="remain">-</span></p>
<p><b>Phase:</b> <span id="phase">-</span> &nbsp; <b>Step:</b> <span id="prog">-</span> &nbsp; <b>Time left in phase:</b> <span id="eta">-</span></p>
<p id="msg"></p>
</div>

<h2>Controller (live)</h2><div class="section">
<b>Set Temperature:</b> <span id="set">-</span> &deg;C &nbsp;
<b>Mode:</b> <span id="mode">-</span> &nbsp;
<b>Pump:</b> <span id="pump">-</span><br>
<b>Frames arriving:</b> <span id="frames">-</span> &nbsp;
<b>Garbled frames rejected:</b> <span id="bad">-</span>
</div>

<h2>Scan Control</h2><div class="section">
Phase <select id="p">
<option value="1">1: every single byte 00-FF</option>
<option value="2">2: Sun frame with new set temp / mode</option>
<option value="3">3: Sun + every byte 00-FF</option>
<option value="4">4: every byte 00-FF + CR</option>
<option value="5">5: letter + new set temp (A-Z, a-z)</option>
</select>
Start at hex <input type="text" id="from" value="00" size="3">
Wait <input type="number" id="wait" value="5" min="2" max="60" style="width:60px"> s<br>
Send first (hex, optional) <input type="text" id="pre" value="" size="10" placeholder="e.g. 52">
<span style="color:#777;font-size:0.85em">sent before each command whenever the controller has dropped out of the mode it produces; also used by Resend and manual send</span><br>
<label><input type="checkbox" id="soh"> Pause when something works</label>
<label><input type="checkbox" id="all"> Run later phases even after a hit</label><br>
<button class="go" id="bstart" onclick="start()">START SCAN</button>
<button class="stop" id="bstop" onclick="call('/scanstop')">STOP</button>
<button id="bresume" onclick="resume()">RESUME</button>
</div>

<h2>Commands That Worked</h2><div class="section">
<table><thead><tr><th>Command (hex)</th><th>What changed</th><th></th></tr></thead><tbody id="hits"></tbody></table>
<button class="grey" onclick="if(confirm('Clear the saved list?'))call('/scanclear')">Clear list</button>
</div>

<h2>Send Hex Manually</h2><div class="section">
<input type="text" id="hex" placeholder="e.g. 53 75 6E 1C" size="40">
<button onclick="call('/scansendhex?hex='+encodeURIComponent(document.getElementById('hex').value)+pre())">Send &amp; check</button>
</div>

<h2>Other Data From Controller</h2><div class="section">
<p style="margin-top:0;color:#777;font-size:0.85em">Anything received that is not a normal status frame (start-up text, replies, errors). Newest at the end.</p>
<pre id="straytxt" style="background:#eee;padding:10px;border-radius:6px;white-space:pre-wrap;word-break:break-all;min-height:20px"></pre>
<pre id="strayhex" style="background:#eee;padding:10px;border-radius:6px;white-space:pre-wrap;font-size:12px;min-height:20px"></pre>
<button class="grey" onclick="call('/scanclear?stray=1')">Clear</button>
</div>

<h2>Log</h2><div class="section"><pre id="log"></pre></div>

<div class="section"><a class="btn" href="/">Back to Main Page</a></div>
</div>
<script>
function $(i){return document.getElementById(i);}
function esc(s){return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;');}
function call(u){fetch(u).then(function(){poll();});}
function pre(){return '&pre='+encodeURIComponent($('pre').value);}
function opts(){return pre()+'&wait='+$('wait').value+'&stoponhit='+($('soh').checked?1:0)+'&allphases='+($('all').checked?1:0);}
function start(){call('/scanstart?phase='+$('p').value+'&from='+encodeURIComponent($('from').value)+opts());}
function resume(){call('/scanstart?resume=1'+opts());}
function poll(){
 fetch('/scanstatus').then(function(r){return r.json();}).then(function(d){
  $('cmd').textContent=d.cmd; $('note').textContent=d.note;
  $('state').textContent=d.state;
  $('remain').textContent=d.state=='Waiting for result'?d.remain+' s':'-';
  $('phase').textContent=d.phase;
  $('prog').textContent=(d.index+1)+' of '+d.count;
  var left=(d.count-d.index)*(d.wait+1.5);
  $('eta').textContent=d.busy?Math.ceil(left/60)+' min':'-';
  $('msg').textContent=d.msg;
  $('set').textContent=d.set; $('mode').textContent=d.mode; $('pump').textContent=d.pump;
  $('frames').innerHTML=d.frames?'yes':'<span class="warn">NO</span>';
  $('bad').textContent=d.bad;
  $('bstart').disabled=d.busy; $('bstop').disabled=!d.busy; $('bresume').disabled=!d.paused;
  var h='';
  for(var i=0;i<d.hits.length;i++){
   h+='<tr><td class="mono">'+esc(d.hits[i].hex)+'</td><td>'+esc(d.hits[i].desc)+'</td><td><button '+(d.busy?'disabled ':'')+'onclick="call(\'/scanresend?i='+i+'\'+pre())">Resend</button></td></tr>';
  }
  $('hits').innerHTML=h||'<tr><td colspan="3"><i>Nothing found yet.</i></td></tr>';
  $('straytxt').textContent=d.straytxt||'(nothing yet)'; $('strayhex').textContent=d.strayhex;
  $('log').textContent=d.log.slice().reverse().join('\n');
 }).catch(function(){$('msg').textContent='(no reply from the Wemos)';});
}
setInterval(poll,1000); poll();
</script></body></html>)HTML";

void handleScanner() {
  server.send_P(200, "text/html", SCAN_PAGE);
}

// ------------------------------------------------------------
// Setup & loop
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  sunSerial.begin(57600);

  SPIFFS.begin();
  loadMqttConfig();
  loadPoolSelection();
  loadHits();

  // WiFiManager AutoConnect.
  // If the saved network is not there yet (e.g. the router is still
  // booting after a power cut) the setup portal opens. It now closes
  // itself after 3 minutes and the Wemos restarts and tries again,
  // instead of sitting in the portal until someone power-cycles it.
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFiManager wm;
  wm.setConfigPortalTimeout(180);
  if (!wm.autoConnect("SunSwitchController")) {
    delay(1000);
    ESP.restart();
  }

  // Web routes
  server.on("/", handleRoot);
  server.on("/poolsel", handlePoolSel);
  server.on("/sendcmd", handleSendCmd);
  server.on("/mqtt", handleMqttConfig);
  server.on("/ha_discovery", handleHADiscovery);
  server.on("/wifi", handleWifiSettings);
  server.on("/start_portal", handleStartPortal);
  server.on("/scanner", handleScanner);
  server.on("/scanstatus", handleScanStatus);
  server.on("/scanstart", handleScanStart);
  server.on("/scanstop", handleScanStop);
  server.on("/scanresend", handleScanResend);
  server.on("/scansendhex", handleScanSendHex);
  server.on("/scanclear", handleScanClear);

  // HTTP OTA update
  httpUpdater.setup(&server);

  server.begin();

  // Arduino OTA (no password)
  ArduinoOTA.setHostname("SunSwitchWemos");
  ArduinoOTA.begin();
}

void loop() {
  ArduinoOTA.handle();
  wifiWatchdog();
  server.handleClient();
  mqttReconnect();
  mqtt.loop();
  readSunSwitchFrame();
  scanLoop();

  mqttWillReconnect();
  mqttWill.loop();
  checkControllerOffline();

  // Refresh the diagnostics every 30 s so Home Assistant keeps them current
  static unsigned long lastDiag = 0;
  if (millis() - lastDiag >= 30000UL) {
    lastDiag = millis();
    publishDiagnostics();
  }
}
