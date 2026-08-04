# Peripherals

Vigilant Engine includes drivers and integration for common peripherals used across STARSTREAK nodes.

## Supported peripherals

- **WS2812B**: Addressable RGB LEDs (status indication, effects)
- **TCAN4550-Q1**: SPI CAN/CAN-FD controller and transceiver
- **TCAN337**: Classic CAN transceiver using the ESP32-S3 TWAI controller

These peripherals are wrapped behind a common abstraction layer to simplify reuse across multiple firmware targets.

## I2C bus

Vigilant Engine also provides an optional shared I2C master interface for external sensors and peripherals.

For setup and API usage, see the [I2C Interface](./i2c-interface.md) page.

## TCAN4550-Q1 CAN-FD verification

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
5. Attempts external loopback through the integrated transceiver and CAN pins.
6. Enters normal CAN mode and logs every frame accepted into RX FIFO 0.

The default timing is 500 kbit/s arbitration and 2 Mbit/s CAN-FD data. The
global filter accepts standard and extended frames, including non-matching
frames. Because no interrupt pin is assigned, the logging task polls the RX
FIFO over SPI once per FreeRTOS scheduler tick. A CAN-silent indication is
expected and logged once when no other CAN node is connected; it is not treated
as a module fault. Timestamp-counter wraparound is likewise informational.

Internal loopback proves the ESP32-to-SPI connection, controller, and message
RAM. External loopback additionally exercises the transceiver path. A scope or
a second CAN node is still required to validate voltage levels and operation on
a real, correctly terminated multi-node bus. If the module has no onboard CAN
termination, fit the termination required by the module before interpreting an
external-loopback failure.

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

## TCAN337 Classic CAN verification

The TCAN337 is a physical-layer transceiver; the ESP32-S3 TWAI peripheral is
the Classic CAN protocol controller. The application uses this connection:

| Signal | ESP32-S3 GPIO |
| --- | ---: |
| RXD / CAN_RX | 15 |
| TXD / CAN_TX | 16 |

Set `TCAN337_BEACON_ADDRESS` in `main/main.c` to change the boot-test CAN ID,
and set `TCAN337_BITRATE` to the bus arbitration rate. The non-G TCAN337 is
limited to 1 Mbit/s. Standard and extended Classic CAN frames are accepted and
logged with their ID, RTR flag, length, timestamp, and complete payload.

At boot, the ESP32 controller enters self-test and self-reception mode, sends an
eight-byte beacon without requiring another node to acknowledge it, and checks
the received copy. It then recreates the controller in normal CAN mode and
waits for external frames using an interrupt-driven receive queue. Controller
error state, TEC/REC, bus errors, queue loss, and transmit failures are logged;
bus-off recovery starts automatically. Error flags are decoded by name, and a
persistent error is rate-limited to one health report every ten seconds while
its counters continue to accumulate.

The module must hold TCAN337 pin 8 (`S`) low for normal mode. If it is connected
to the ESP32, assign `TCAN337_SILENT_IO` instead of `GPIO_NUM_NC`. TCAN337 pin 5
(`FAULT`) is also supported optionally through `TCAN337_FAULT_IO`; its
open-drain output requires the external pull resistor specified by TI. With
only RXD and TXD connected, transceiver-specific undervoltage, thermal, and
dominant-timeout faults cannot be read directly, but CAN controller errors are
still reported.

As with every high-speed CAN network, use correct CANH/CANL termination. The
self-test verifies controller operation and self-reception, while a second node
or oscilloscope is still required to validate interoperability and physical bus
levels.

A repeated `stuff` error with TEC at zero and an increasing REC means RXD is
seeing malformed activity from the physical bus; it is not a failed loopback
test. On an unconnected bench setup, check that `S` is low, TCAN337 and ESP32
grounds are common, the transceiver supply is valid, and CANH/CANL are
terminated. With the system powered down, a normal bus with one 120-ohm
terminator at each end measures about 60 ohms between CANH and CANL.

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
