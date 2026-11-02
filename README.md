# SunSwitch Reader

An ESP8266 (Wemos D1 Mini) bridge that reads live data from a **SunSwitch solar pool heating controller** through its serial diagnostic port and publishes it to **Home Assistant over MQTT**, with auto-discovery, a built-in web interface and OTA updates.

> Unofficial hobby project. Not affiliated with or endorsed by SunSwitch. Connecting anything to your controller is at your own risk.

---

## Features

- **Decodes the SunSwitch status frame**: the 23-byte `Sun…` frame at 57,600 baud
  - Set temperature
  - Pool sensor 1 and pool sensor 2 temperature (you pick which one goes to Home Assistant)
  - Roof temperature
  - Pump status (on/off)
  - Controller mode: Automatic, Manual, Winter Mode, Set Temp Mode, Continuous Manual, Controller Off, plus *(Energy Saving)*
  - Controller runtime
- **Glitch filtering**: a value is only published once two good frames in a row agree, and frames that fail sanity checks are dropped. A single corrupted frame never reaches Home Assistant.
- **Home Assistant MQTT auto-discovery**: no YAML needed. The entities all appear under one *SunSwitch Controller* device.
- **Offline detection**: the controller mode reports `Offline` if the Wemos loses power (via MQTT Last Will) or if the controller stops sending frames for 60 s. Roof temperature and pump status then show as unavailable.
- **Diagnostics in HA**: IP address, Wemos uptime and a sensor-data-error flag
- **Web interface**: live values, WiFi signal, raw frame viewer, debug byte table, MQTT settings, pool sensor selector and a manual command sender
- **Command scanner** (`/scanner`): an experimental tool for working out what commands the controller accepts. It sends candidate commands one at a time, watches for changes to set temp, mode or pump, and saves anything that works.
- **WiFiManager**: captive portal for first-time WiFi setup. The portal times out after 3 minutes so the device never gets stuck in it after a power cut.
- **Robust**: WiFi watchdog (reconnects, and restarts after 10 minutes offline), MQTT reconnect backoff, non-blocking loop
- **OTA updates**: ArduinoOTA plus a browser upload page at `/update`

---

## Hardware

| Item | Notes |
|---|---|
| Wemos D1 Mini (ESP8266) | Any ESP8266 board should work with pin changes |
| Connection to the SunSwitch serial port | RX, TX and GND |
| Power | The SunSwitch 5 V rail  |

### Wiring

| Wemos pin | Connects to |
|---|---|
| D1 (GPIO5) | SunSwitch TX → Wemos **RX** |
| D2 (GPIO4) | SunSwitch RX ← Wemos **TX** (only needed for sending commands) |
| GND | SunSwitch GND |
| 5V | SunSwitch 5 V (optional) |

> ⚠️ The ESP8266 runs at 3.3 V. Check your controller's serial voltage before wiring, and use a level shifter or divider if it is 5 V.
>
> The sketch uses SoftwareSerial. Hardware serial was tried, but it held the line high and no data came through.

---

## Software

### Arduino IDE setup

1. Install the **ESP8266 board package** (Boards Manager → *esp8266 by ESP8266 Community*).
2. Install these libraries through the Library Manager:
   - **WiFiManager** by tzapu
   - **PubSubClient** by Nick O'Leary
3. The ESP8266 core already includes `ESP8266WiFi`, `ESP8266WebServer`, `ESP8266HTTPUpdateServer`, `ArduinoOTA`, `SoftwareSerial` and `FS` (SPIFFS).
4. Select **LOLIN(WEMOS) D1 R2 & mini** and choose a flash size that includes a filesystem (e.g. *4MB (FS:1MB)*).
5. Upload the sketch.

### First-time setup

1. When it powers up for the first time, the Wemos opens a WiFi access point named **`SunSwitchController`**.
2. Connect to it, choose your WiFi network and enter the password.
3. Once it's on your network, browse to the Wemos's IP address.
4. Go to **MQTT Configuration** and enter your broker host, port, username and password. Save.
5. The Home Assistant discovery messages are published automatically, and the device shows up under *Settings → Devices & Services → MQTT*.

No credentials are stored in the code. WiFi details are held by WiFiManager, and the MQTT settings are saved to SPIFFS (`/mqtt.cfg`).

---

## MQTT topics

| Topic | Payload | Retained |
|---|---|---|
| `sunswitch/set_temp` | °C | yes |
| `sunswitch/pool_temp` | °C (selected sensor) | yes |
| `sunswitch/roof_temp` | °C | yes |
| `sunswitch/pump_status` | `ON` / `OFF` | yes |
| `sunswitch/controller_mode` | e.g. `Automatic`, `Manual (Energy Saving)`, `Offline` | yes |
| `sunswitch/runtime` | seconds | yes |
| `sunswitch/pool_error` | `ON` / `OFF` (3+ garbled frames in a row) | yes |
| `sunswitch/ip_address` | IP address | yes |
| `sunswitch/esp_uptime` | `Nd HH:MM:SS` | yes |
| `sunswitch/status` | `online` / `offline` (LWT) | yes |

You can change the main topics on the MQTT Configuration page.

---

## SunSwitch frame format (as decoded so far)

Every frame is 23 bytes. It starts with `53 75 6E` ("Sun") and ends with `0A 0D`.

| Byte(s) | Meaning |
|---|---|
| 0–2 | `Sun` header |
| 4–5 | Runtime counter (seconds, big-endian) |
| 8 | Energy-saving timer (above 0 while in Energy Saving) |
| 10 | Set temperature (°C) |
| 11 | Pool sensor 1 (°C) |
| 12 | Pool sensor 2 (°C) |
| 13 | Roof temperature (°C) |
| 14 | Controller mode |
| 15 | Pump: `00` off, `01` on |
| 21–22 | `0A 0D` terminator |

**Mode byte (14):**

| Value | Mode |
|---|---|
| `0x03` | Winter Mode |
| `0x08` | Controller Off |
| `0x09` | Automatic |
| `0x11` | Manual |
| `0x49` | Set Temp Mode |
| `0x91` | Continuous Manual |

The other bytes are still unknown. Contributions welcome!

---

## Web pages

| Path | Purpose |
|---|---|
| `/` | Status, sensors, raw frame, debug bytes, pool sensor selector, command sender |
| `/mqtt` | MQTT settings, connection status, re-publish HA discovery |
| `/scanner` | Command scanner |
| `/update` | Firmware upload (OTA) |
| `/wifi` | Start the WiFiManager portal |

---

## Security notes

This firmware is designed for a trusted home network:

- The web interface and the `/update` firmware page have **no login**.
- ArduinoOTA has **no password**.
- Anyone who can open the MQTT page can see the saved MQTT password in the page source.

If that matters on your network, keep the device on an isolated IoT VLAN, or add authentication (`httpUpdater.setup(&server, "/update", user, pass)` and `ArduinoOTA.setPassword(...)`).

---

## Changelog

- **V3.12**: Glitch filtering (two matching frames needed), roof temps above 99 rejected, runtime must tick forward sensibly, glitch counter on the main page
- **V3.11**: Roof temp and pump status go unavailable in HA while the mode is `Offline`
- **V3.10**: Controller mode shows `Offline` when the SunSwitch or Wemos is powered off
- **V3.9**: IP address, uptime and sensor data error published to HA again
- **V3.8**: Energy Saving shown in the mode, WiFi auto-reconnect and self-restart, no more hanging in the WiFi portal after a power cut

---
