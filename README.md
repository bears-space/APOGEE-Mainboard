# Vigilant Engine

Vigilant Engine is the framework on which all ESP-based nodes of the **Starstreak** project are built.  
It provides the fundamental building blocks for connected, remotely maintainable devices, including
reliable OTA updates, flexible network modes, and peripheral support.

## Core Features

- **OTA (Over-The-Air updates)**
  - Firmware updates via HTTP/Web UI
  - Designed for safe, recoverable updates on ESP targets

- **Network Modes**
  - `AP` – Device creates its own access point
  - `STA` – Device connects to an existing Wi-Fi network
  - `APSTA` – Dual mode (access point + station)

- **Webserver**
  - Serves configuration & status pages
  - Handles OTA firmware upload
  - Endpoint structure designed to be extendable per project

- **Debugging**
  - Wireless debugging over the network (e.g. TCP terminal)
  - Centralized logging API for modules
  - Optional verbose / debug builds
## Peripherals
Vigilant Engine includes drivers / integration for:

- **WS2812B** – Addressable RGB LEDs (status indication, effects)
- **TCAN4550-Q1** – SPI CAN controller
- **TCAN332** – CAN transceiver

These peripherals are wrapped in a common abstraction layer so they can be reused across all Starstreak nodes.

## Partition setup and the Flasher
Vigilant engine is made from two main partitions, the factory partition, and the ota_0 partition.

## Flashing the ESP32
To flash the main programm, you first need to flash the recovery partition. This part of the project can be found in the ```/vigilant-engine-recovery``` directory. You can use ESP-IDF's built in ```"Build, Flash and Monitor"``` action to flash to the ESP. After flashing, the ESP will open an AP, with the SSID being ```"VE-Recovery"```, the password is ```"starstreak"```. When you connected successfully, the webserver should be at ```http://192.168.4.1/```. You then have the option to upload a binary, and then the ability to flash. When you have selected a valid .bin file, you can press "Upload" to Flash the ESP. When the ESP accepts the binary file, it will return ```"OTA OK: wrote %d bytes. Rebooting to ota_0…"```
The ESP will then boot into the ota_0 partition. Vigilant engine has has route built into the webserver called ```/rebootfactory```, which enables you to boot into the recovery app, allowing you to flash new firmware.
