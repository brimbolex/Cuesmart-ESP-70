# CueSmart ESP-70

CueSmart ESP-70 is an ESP32-S3-based Ethernet-to-CAN gateway for controlling and monitoring d&b amplifiers through R1. It implements an R70-compatible network interface and translates requests between Ethernet and the amplifier CAN bus.

The firmware targets the ESP32-S3-ETH board with a W5500 Ethernet interface. It includes a browser-based configuration interface, DHCP or static IP addressing, and four GPI inputs with configurable amplifier preset routes.

## R1 timeout notice

Due to the ESP32-based architecture and its shared processing of Ethernet, CAN communication, and other firmware tasks, **rare timeouts may appear in R1**. Please take this limitation into account when using the gateway; timeout-free operation is not guaranteed.

## Network configuration

The factory configuration uses the static IP address **192.168.254.70**, subnet mask **255.255.255.0**, and gateway **192.168.254.1**. Open `http://192.168.254.70` from a computer on the same subnet to access the configuration interface. DHCP can be enabled there.

## Onboard RGB status LED

Only the onboard RGB LED on GPIO 21 is used.

| State | Color | RGB |
| --- | --- | --- |
| Ethernet failure or no IP address | Red | 255, 0, 0 |
| DHCP address active | Cyan | 0, 255, 255 |
| Static IP active, including DHCP fallback | Blue | 0, 0, 255 |
| CAN receive or transmit activity | Short green flash | 0, 255, 0 |
| Factory reset | Three violet flashes | 128, 0, 255 |

Ethernet failure takes priority over CAN activity. Between CAN flashes, the LED returns to the current network status color.

A factory reset can be triggered through the web interface or by holding the factory-reset input on GPIO 4 low for five seconds. It resets network and GPI settings, flashes the LED three times, and restarts the device.

## Build

The firmware has been compiled for `esp32:esp32:esp32s3` using ESP32 Arduino core **3.3.8**. The repository includes the project library and the required Ethernet **2.0.2** and Adafruit NeoPixel **1.15.4** libraries.

With Arduino CLI and the ESP32 core installed, run:

```sh
./build.sh
```

The main sketch is `CueSmart-ESP-70/CueSmart-ESP-70.ino`. Building does not upload the firmware to the board.

## Serial diagnostics

Serial diagnostic output and test commands are disabled by default. To restore them, set `ENABLE_SERIAL_DIAGNOSTICS` to `1` in `CueSmart-ESP-70/SerialDiagnostics.h`, then rebuild and upload the firmware. Serial diagnostics use **115200 baud**.

## Revision

The **rev1** tag marks the initial firmware snapshot, including the onboard LED behavior and disabled serial diagnostics. Later documentation updates are available on `main`.

## No warranty and limitation of liability

This project, including its firmware, source code, and documentation, is provided **“AS IS” and “AS AVAILABLE”**, without warranty of any kind, express or implied, including warranties of merchantability, fitness for a particular purpose, and non-infringement. No guarantee is made regarding reliability, compatibility, uninterrupted operation, or freedom from errors or timeouts.

Use this project at your own risk. You are responsible for testing and validating it with your equipment before use, including in live or production environments.

To the maximum extent permitted by applicable law, the authors and contributors shall not be liable for any claims, damages, or other liability arising from the use of, or inability to use, this project, including equipment damage, data loss, loss of revenue, or interruption of operations, whether in contract, tort, or otherwise. Nothing in this disclaimer excludes or limits liability that cannot legally be excluded or limited.
