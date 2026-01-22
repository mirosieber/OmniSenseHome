# OmniSensHome

**A Comprehensive Smart Home Multi-Sensor Platform Based on ESP32-C6 with Zigbee Communication**

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ESP-IDF](https://img.shields.io/badge/ESP--IDF-v5.x-orange.svg)](https://github.com/espressif/esp-idf)
[![Platform](https://img.shields.io/badge/Platform-ESP32--C6-green.svg)](https://www.espressif.com/en/products/socs/esp32-c6)

## Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [Hardware](#hardware)
  - [Supported Sensors](#supported-sensors)
  - [Supported Actuators](#supported-actuators)
  - [Pin Configuration](#pin-configuration)
- [Software Architecture](#software-architecture)
  - [FreeRTOS Tasks](#freertos-tasks)
  - [Zigbee Endpoints](#zigbee-endpoints)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Building the Project](#building-the-project)
  - [Flashing the Firmware](#flashing-the-firmware)
- [Configuration](#configuration)
  - [Config File Structure](#config-file-structure)
  - [Uploading Config Files](#uploading-config-files)
- [Zigbee Integration](#zigbee-integration)
  - [Connecting to Zigbee2MQTT](#connecting-to-zigbee2mqtt)
  - [OTA Updates](#ota-updates)
- [Features](#features)
  - [RGB LED Status Indicator](#rgb-led-status-indicator)
  - [Intruder Alarm System](#intruder-alarm-system)
  - [Contact Switches](#contact-switches)
  - [Relay Control](#relay-control)
- [Development](#development)
  - [Project Structure](#project-structure)
  - [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgments](#acknowledgments)

---

## Overview

OmniSensHome is a versatile, modular smart home sensor platform built on the ESP32-C6 microcontroller that communicates via **Zigbee 3.0**. The project allows you to create custom multi-sensor devices that can monitor various environmental parameters, control actuators, and integrate seamlessly with home automation systems like **Zigbee2MQTT** and **Home Assistant**.

The system is designed with flexibility in mind - a single firmware can support multiple hardware configurations through a JSON-based configuration file. This approach eliminates the need to recompile code for different sensor combinations.

**Created by:** Miro Sieber  
**Last Updated:** January 2026

---

## Key Features

✨ **Multi-Sensor Support**
- Temperature monitoring (MTS4Z, AHT21)
- Humidity sensing (AHT21)
- Air quality monitoring (ENS160: eCO2, TVOC, AQI)
- Ambient light sensing (OPT3004)
- Sound level monitoring (dB sensor)
- Presence detection (LD2412 radar sensor)
- Contact switches for doors/windows

🔧 **Flexible Actuator Control**
- Up to 4 relay outputs (NO/NC supported)
- RGB LED status indicator with automatic brightness adjustment
- Audio speaker for alarms/notifications
- Buzzer for alerts

🌐 **Zigbee 3.0 Integration**
- Native Zigbee 3.0 support via ESP32-C6
- Works as router (always-on) or end-device (battery-powered)
- Over-The-Air (OTA) firmware updates with delta compression
- Seamless integration with Zigbee2MQTT

📝 **Configuration-Driven Design**
- JSON-based hardware configuration
- Single firmware for multiple hardware variants
- No code recompilation needed for different sensor setups

🔒 **Security Features**
- Intruder alarm system
- Door/window contact monitoring
- Presence-based security
- Bluetooth control for LD2412 sensor

---

## Hardware

### Supported Sensors

| Sensor | Type | Measurement | Update Interval | Interface |
|--------|------|-------------|-----------------|-----------|
| **OPT3004** | Illuminance | 0-83,000 Lux (0.01 Lux resolution) | 100 ms | I²C |
| **MTS4Z** | Temperature | ±0.1°C accuracy | 5 seconds | I²C |
| **AHT21** | Temperature & Humidity | ±0.5°C, ±3% RH | 10 seconds | I²C |
| **ENS160** | Air Quality | eCO2, TVOC, AQI | 10 seconds | I²C |
| **dB Sensor** | Sound Level | Noise level in dB | 100 ms | Analog/I²S |
| **LD2412** | Presence/Motion | Human presence detection | 100 ms | UART |
| **Contact Switches** | Binary | Door/window status | Event-driven | GPIO |

### Supported Actuators

| Actuator | Type | Description | Interface |
|----------|------|-------------|-----------|
| **Relays (1-4)** | Switching | NO (Normally Open) or NC (Normally Closed) | GPIO |
| **RGB LED** | Status Indicator | Air quality visualization with auto-brightness | GPIO (PWM) |
| **Speaker** | Audio Output | Alarm tones and notifications | I²S |
| **Buzzer** | Audio Alert | Simple alarm tones | GPIO |

### Pin Configuration

The ESP32-C6 has limited external GPIO pins available for peripherals:

**Available External GPIOs:** 2, 3, 14, 15

**Fixed I²C Pins:**
- SDA: Configurable in config file
- SCL: Configurable in config file

**I²S Pins (Speaker/Microphone):**
- LRC, BCLK, DIN: Configurable (note: some pins shared with microphone)

**Reserved Pins:**
- Pin 1: ENS160 interrupt (unused in software, can be repurposed)

⚠️ **Important:** Ensure no GPIO pin is assigned to multiple functions to avoid hardware conflicts or damage!

---

## Software Architecture

OmniSensHome is built on **FreeRTOS** and utilizes a multi-tasking architecture where each sensor and actuator runs in its own dedicated task. The system is highly modular and only creates tasks for sensors/actuators that are enabled in the configuration file.

### FreeRTOS Tasks

| Task Name | Description | Always Running? |
|-----------|-------------|-----------------|
| **MainTask** | System initialization, Zigbee factory reset button monitoring | ✅ Yes |
| **ZigbeeMain** | Zigbee communication, OTA updates, routing | ✅ Yes |
| **Lux Sensor Task** | OPT3004 sensor reading, RGB LED brightness adjustment | Only if enabled |
| **Temperature Task** | MTS4Z sensor reading | Only if enabled |
| **Temp/Humidity/AQ Task** | AHT21 + ENS160 sensor reading, air quality monitoring | Only if enabled |
| **Sound Level Task** | dB sensor reading | Only if enabled |
| **Presence Task** | LD2412 radar sensor data processing | Only if enabled |
| **UART Event Task** | LD2412 UART communication | Only if LD2412 enabled |
| **UART TX Task** | LD2412 command transmission | Only if LD2412 enabled |
| **UART RX Task** | LD2412 data reception | Only if LD2412 enabled |
| **Contact Switch Task** | Door/window contact monitoring | Only if enabled |
| **Relay Control Task** | Relay output control, button/switch handling | Only if enabled |
| **Speaker Task** | Alarm audio playback | Only if enabled |
| **Buzzer Task** | Alarm tone generation | Only if enabled |

### Zigbee Endpoints

The system uses multiple Zigbee endpoints to organize different functionalities:

| Endpoint | Function | Cluster Type |
|----------|----------|--------------|
| 1 | Range Extender (message routing) | Router |
| 2 | Illuminance Sensor (Lux) | Illuminance Measurement |
| 3 | Temperature Sensor (OTA capable) | Temperature Measurement |
| 4 | Temperature & Humidity Sensor | Temperature + Humidity |
| 5 | eCO2 Sensor | Carbon Dioxide Measurement |
| 6 | Sound Level Sensor (dB) | Analog Input |
| 7 | Presence Sensor | Occupancy Sensing |
| 8-11 | Contact Switches 1-4 | Binary Input |
| 12-15 | Relays 1-4 | On/Off Output |
| 16 | TVOC Sensor | Carbon Dioxide Measurement |
| 17 | RGB LED | Dimmable Light |
| 18 | LD2412 Bluetooth Control | On/Off |
| 19 | Audio Gong Trigger | On/Off |
| 20 | Intruder Alarm Activate | On/Off |
| 21 | Intruder Detected Status | Binary Sensor |
| 22 | Air Quality Index | Carbon Dioxide Measurement |
| 23 | Zigbee Device Reset | On/Off |
| 24 | Alarm Trigger | On/Off |

---

## Getting Started

### Prerequisites

**Hardware:**
- ESP32-C6 development board or custom OmniSensHome board
- USB cable for programming
- Sensors and actuators as needed

**Software:**
- [ESP-IDF v5.x](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/get-started/)
- [Python 3.x](https://www.python.org/)
- Git

**Optional:**
- [Zigbee2MQTT](https://www.zigbee2mqtt.io/) for Zigbee integration
- [ESP32DataFlasher](https://github.com/mirosieber/ESP32DataFlasher) for configuration file upload

### Building the Project

1. **Clone the repository:**
   ```bash
   git clone https://github.com/mirosieber/OmniSensHome.git
   cd OmniSensHome
   ```

2. **Set up ESP-IDF environment:**
   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```

3. **Configure the project:**
   ```bash
   idf.py menuconfig
   ```
   
   ⚠️ **Important:** Ensure `CONFIG_FREERTOS_HZ` is set to **1000 Hz** (not 100 Hz) in `sdkconfig`, otherwise compilation will fail.

4. **Build the firmware:**
   ```bash
   idf.py build
   ```

### Flashing the Firmware

1. **Connect the ESP32-C6 via USB**

2. **Flash the firmware:**
   ```bash
   idf.py -p /dev/ttyUSB0 flash
   ```

3. **Monitor the output (optional):**
   ```bash
   idf.py -p /dev/ttyUSB0 monitor
   ```

---

## Configuration

OmniSensHome uses a JSON configuration file stored in the device's SPIFFS partition. This allows a single firmware image to support multiple hardware configurations.

### Config File Structure

The configuration file defines:
- **Device Information:** Type (Router/End Device), manufacturer, model, versions
- **Network:** WiFi credentials (if needed)
- **Factory Reset Button:** GPIO pin assignment
- **I²C Configuration:** SDA/SCL pins
- **RGB LED:** Red, Green, Blue GPIO pins
- **Speaker:** I²S pins and configuration
- **Buzzer:** GPIO pin
- **Microphone:** I²S pins (not yet implemented in v8)
- **Display:** Configuration (not yet implemented in v8)
- **Sensors Array:** List of sensors with type, I²C address, pins, etc.
- **Switches Array:** Up to 4 switches (Button, Switch, Contact types)
- **Relays Array:** Up to 4 relays (NORelay, NCRelay types)

**Example Config Structure (see `main/configLoader.h`):**
```c
typedef struct {
  device_info_t device;
  wifi_config_t wifi;
  rgb_led_config_t rgb_led;
  i2c_config_t i2c;
  speaker_config_t speaker;
  buzzer_config_t buzzer;
  sensor_config_t sensors[MAX_SENSORS];
  switch_config_t switches[MAX_SWITCHES];
  relay_config_t relays[MAX_RELAYS];
  // ...
} app_config_t;
```

### Switch/Button Types

- **Button:** Momentary push button (sends pulse on press)
- **Switch:** Toggle switch (state changes on each press)
- **Contact:** Door/window contact sensor (pulled to GND when closed)

### Relay Types

- **NORelay:** Normally Open relay (closes contact when activated)
- **NCRelay:** Normally Closed relay (opens contact when activated - inverse logic)

### Uploading Config Files

Configuration files are uploaded using the [ESP32DataFlasher](https://github.com/mirosieber/ESP32DataFlasher) tool:

1. Connect device via USB
2. Use ESP32DataFlasher to write the JSON config to SPIFFS
3. Restart the device

⚠️ **When changing hardware configuration (adding/removing sensors or contacts), you must:**
1. Erase the entire flash memory
2. Upload the new configuration file
3. Flash the firmware
4. Reconnect the device to Zigbee network (factory reset)

---

## Zigbee Integration

### Connecting to Zigbee2MQTT

**First-Time Connection:**

1. Power on the OmniSensHome device
2. The RGB LED should blink white (pairing mode)
3. If not blinking white, press the factory reset button
4. In Zigbee2MQTT web interface, enable "Permit Join"
5. The device will automatically connect and be interviewed

**After Major Updates or Config Changes:**

If endpoints have been added/modified or the configuration file changed:

1. Factory reset the device (button press or remote via Zigbee2MQTT)
2. Device will restart and blink white
3. Enable "Permit Join" in Zigbee2MQTT
4. Re-interview the device in Zigbee2MQTT interface

### OTA Updates

OmniSensHome supports **Delta OTA updates** for fast, efficient firmware updates over Zigbee.

**Configuration (Zigbee2MQTT):**

Add to `configuration.yaml`:
```yaml
ota:
  update_check_interval: 2880
  zigbee_ota_override_index_location: https://raw.githubusercontent.com/mirosieber/OmniSensHome/refs/heads/main/my_index.json
```

**Creating a New OTA Release:**

1. Increment `OTA_UPGRADE_RUNNING_FILE_VERSION` and `OTA_UPGRADE_DOWNLOADED_FILE_VERSION` in `main/main.cpp` (by 0x1 each)
2. Build the project: `idf.py build`
3. Test via USB to ensure functionality
4. Commit and push changes to GitHub
5. GitHub Actions will automatically:
   - Extract version information
   - Create delta OTA file (difference from previous version)
   - Generate OTA firmware file (`.ota`)
   - Update `my_index.json` with new release info
   - Create a GitHub release
6. Update devices via Zigbee2MQTT web interface (one at a time)

**OTA Update Time:**
- Full OTA: ~3 hours
- Delta OTA: ~3 minutes ⚡

The delta OTA feature significantly reduces update time by only transmitting the differences between firmware versions.

---

## Features

### RGB LED Status Indicator

The RGB LED provides visual feedback about air quality:

| Color | Air Quality | Description |
|-------|-------------|-------------|
| 🔵 Blue | Excellent | Very good air quality |
| 🟢 Green | Good | Good air quality |
| 🟡 Yellow | Moderate | Moderate air quality |
| 🔴 Red | Poor | Poor air quality |
| 🟣 Purple | Very Poor | Very poor air quality |
| ⚪ White (blinking) | - | Zigbee pairing mode |

**Auto-Brightness:**
- LED brightness automatically adjusts based on ambient light (OPT3004 sensor)
- Turns off at night for comfortable sleeping
- Brightness can be fine-tuned via Zigbee commands

### Intruder Alarm System

OmniSensHome can function as a security alarm system:

**Features:**
- Activate/deactivate via Zigbee endpoint 20
- Monitors contact switches (doors/windows)
- Monitors presence sensor (LD2412)
- Triggers audio alarm (speaker or buzzer) when intrusion detected
- Sends alert to Zigbee coordinator
- Status reported via endpoint 21

**Acoustic Alarm:**
- Speaker plays WAV file alarm tone
- Buzzer plays hardcoded alarm tone
- Can be triggered directly via Zigbee (endpoint 24)

### Contact Switches

Up to 4 contact switches for monitoring doors and windows:
- Pull-up resistor enabled
- Switch must pull pin to GND when closed
- Reports status via Zigbee endpoints 8-11

### Relay Control

Up to 4 relays can be controlled:
- Supports both NO (Normally Open) and NC (Normally Closed) relays
- Can be paired with buttons/switches automatically
- Handles both momentary buttons and toggle switches
- Reports status via Zigbee endpoints 12-15

**Automatic Pairing:**
The relay control task automatically pairs active relays with active buttons/switches in sequential order. For example, if Button 2 and Relay 4 are the only active ones, they will be paired together.

---

## Development

### Project Structure

```
OmniSensHome/
├── main/                      # Main application code
│   ├── main.cpp              # Main entry point and Zigbee setup
│   ├── configLoader.c/h      # Configuration file parser
│   ├── Sensoren.cpp/h        # Sensor management
│   ├── RGB.cpp/h             # RGB LED control
│   ├── Audio.cpp/h           # Audio/speaker management
│   ├── DbSensor.cpp/h        # Sound level sensor
│   ├── TaskMonitor.cpp/h     # FreeRTOS task monitoring
│   └── ...
├── components/               # External components and libraries
│   ├── SparkFun_ENS160/     # ENS160 air quality sensor
│   ├── arduino-esp32-3.2.1/  # Arduino ESP32 framework
│   └── ...
├── Dokumentation/            # German documentation
│   ├── Software_Struktur.md  # Software architecture
│   ├── Sensoren.md           # Sensor documentation
│   ├── Config_File.md        # Config file format
│   ├── OTA.md                # OTA update process
│   └── ...
├── OTAcreate/                # OTA file generation scripts
├── .github/workflows/        # GitHub Actions for automated OTA
├── CMakeLists.txt            # CMake build configuration
├── sdkconfig                 # ESP-IDF SDK configuration
├── partitions.csv            # Flash partition table
└── README.md                 # This file
```

### Troubleshooting

**Compilation Issues:**

1. **FreeRTOS tick rate error:**
   - Ensure `CONFIG_FREERTOS_HZ=1000` in `sdkconfig`
   - If not set, run `idf.py menuconfig` → Component config → FreeRTOS → Tick rate Hz

2. **Build failures:**
   ```bash
   idf.py fullclean
   # If that doesn't work:
   rm -rf managed_components
   idf.py clean
   idf.py build
   ```

3. **Component dependency issues:**
   ```bash
   rm dependencies.lock
   idf.py reconfigure
   idf.py build
   ```

**Runtime Issues:**

1. **Device not pairing with Zigbee:**
   - Press factory reset button
   - Check if LED is blinking white
   - Ensure Zigbee2MQTT has "Permit Join" enabled

2. **Config file not loading:**
   - Verify JSON format is correct
   - Check file size matches partition table
   - Re-upload config using ESP32DataFlasher

3. **Sensors not working:**
   - Verify I²C connections (SDA/SCL)
   - Check sensor addresses match config file
   - Enable I²C in config file
   - Verify GPIO pins are not conflicting

**Debugging:**

Enable verbose logging by connecting via USB:
```bash
idf.py monitor -p /dev/ttyUSB0
```

---

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

**Guidelines:**
1. Fork the repository
2. Create a feature branch
3. Test your changes thoroughly via USB before committing
4. Ensure code compiles without warnings
5. Update documentation if needed
6. Submit a pull request

---

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE.md) file for details.

---

## Acknowledgments

- **Espressif Systems** for the ESP32-C6 and ESP-IDF framework
- **Zigbee2MQTT** community for excellent home automation integration
- **SparkFun** for the ENS160 library
- **Arduino** community for ESP32 Arduino framework

---

## Contact

**Author:** Miro Sieber  
**Repository:** [github.com/mirosieber/OmniSensHome](https://github.com/mirosieber/OmniSensHome)

For issues and feature requests, please use the [GitHub Issues](https://github.com/mirosieber/OmniSensHome/issues) page.

---

## Additional Resources

- [ESP32-C6 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/)
- [Zigbee2MQTT Documentation](https://www.zigbee2mqtt.io/)
- [ESP-IDF Programming Guide](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [FreeRTOS Documentation](https://www.freertos.org/Documentation/RTOS_book.html)

---

*Last Updated: January 2026*
