<!--
# SPDX-AI-Disclosure: ai-assisted
# SPDX-AI-Model: gpt-6
# SPDX-AI-Provider: OpenAI
# SPDX-AI-Scope: Documentation updates and loopback behavior corrections.
# SPDX-AI-Date: 2026-10-13
-->

# Peripherals

Vigilant Engine includes drivers and integration for common peripherals used across STARSTREAK nodes.

## Supported peripherals

- **WS2812B**: Addressable RGB LEDs (status indication, effects)
- **TCAN4550-Q1**: SPI CAN/CAN-FD controller and transceiver

These peripherals are wrapped behind a common abstraction layer to simplify reuse across multiple firmware targets.

## I2C bus

Vigilant Engine also provides an optional shared I2C master interface for external sensors and peripherals.

For setup and API usage, see the [I2C Interface](./i2c-interface.md) page.

## TCAN4550-Q1 CAN-FD verification

The driver now uses the TI TCAN4550 library with an ESP-IDF SPI backend. Source,
BSD-3-Clause notices, API details, and validation instructions are documented in
[`components/tcan4550/README.md`](https://github.com/bears-space/APOGEE-Mainboard/blob/main/components/tcan4550/README.md).

The main application contains a boot-time hardware check and a continuous CAN
frame logger for this ESP32-S3 connection:

| Signal | ESP32-S3 GPIO |
| --- | ---: |
| nCS | 10 |
| SDI / MOSI | 11 |
| SCLK | 12 |
| SDO / MISO | 13 |

Set `TCAN4550_BEACON_ADDRESS` near the top of `main/main.c` to change the CAN
identifier used by the self-test beacon. Values through `0x7FF` use a standard
11-bit identifier; values through `0x1FFFFFFF` use an extended identifier. Also
set `TCAN4550_OSCILLATOR_HZ` to match the board's 20 MHz or 40 MHz TCAN4550
crystal/clock.

At boot, the firmware performs these checks:

1. Reads the `TCAN4550` identity and M_CAN endian registers over SPI.
2. Writes and reads back the device scratch register.
3. Clears and configures all 2 KiB of ECC-protected message RAM.
4. Sends and receives a 16-byte, bit-rate-switched CAN-FD frame in internal
   loopback.
5. Optionally runs external loopback when explicitly enabled for a test bus.
6. Enters normal CAN mode and logs every frame accepted into RX FIFO 0.

The default timing is 500 kbit/s arbitration and 2 Mbit/s CAN-FD data. The
global filter accepts standard and extended frames, including non-matching
frames. Because no interrupt pin is assigned, the logging task polls the RX
FIFO over SPI once per FreeRTOS scheduler tick. A CAN-silent indication is
expected and logged once when no other CAN node is connected; it is not treated
as a module fault. Timestamp-counter wraparound is likewise informational.

Internal loopback tests the ESP32-to-SPI connection, controller, and message
RAM. External loopback internally feeds TX back to RX while driving the CAN
pins; it does not verify the physical receive path. It is disabled by default
because it transmits a CAN-FD beacon onto the connected bus. A scope or a second
CAN-FD node is required to validate the transceiver and a correctly terminated
multi-node bus. Initialization starts passive monitoring; the example explicitly
enables normal operation after internal loopback.

Build, flash, and open the serial monitor with:

```sh
idf.py -p <serial-port> flash monitor
```

Successful startup includes `SPI verified`, `Internal loopback passed`, and
`CAN receiver started in normal mode`. Incoming frames are logged with their
identifier type, CAN/CAN-FD format, BRS/RTR/ESI flags, length, timestamp,
filter information, and the complete payload. `FIFO_LOSS` in a frame log means
the serial logger could not drain the receive FIFO quickly enough and one or
more frames were lost.

### Status LED

Depending on the mode, the status LED gives information about the status of the device differently.

Supported LEDs: Generic 1-Pin, Generic RGB, WS2812B

For more information about the available settings, see the menuconfig and the [config page](./config.md).

#### RGB-Mode
- **Green (slow) blinking**: Info, Period is 1s
- **Blue (faster) blinking**: Warning, Period is 600ms
- **Red (fast) blinking**: Error, Period is 300ms

#### Blink-Mode (Generic 1-Pin only)
- **Slow blinking**: Info, Period is 2s
- **Faster blinking**: Warning, Period is 700ms
- **Fast blinking**: Error, Period is 100ms
