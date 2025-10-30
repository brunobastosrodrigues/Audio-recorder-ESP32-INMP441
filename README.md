# ESP32-INMP441

A project combining ESP32 with INMP441 I2S microphone featuring BLE GATT (IMA-ADPCM), WAV recording (SPIFFS), and HTTP file browser functionality.

## Features
- BLE GATT with IMA-ADPCM encoding
- WAV recording to SPIFFS
- HTTP file browser
- Push-to-record functionality (hold BOOT button)
- Silence watchdog
- Gain control
- Wi-Fi hotspot controls

## Quick Controls
Connect via Serial (115200 baud):

| Key | Function |
|-----|----------|
| `1` | LEFT/STD mode |
| `2` | RIGHT/STD mode |
| `3` | LEFT/MSB mode |
| `4` | RIGHT/MSB mode |
| `+` | Increase volume (lower SHIFT) |
| `-` | Decrease volume (raise SHIFT) |
| `r` | Start/stop recording (manual) |
| `h` | Help |
| `w` | Toggle Wi-Fi AP on/off |
| `W` | Force AP ON |
| `X` | Force AP OFF |

## HTTP File Browser Access
- SSID: `ESP32-Recorder`
- Password: `micglass`
- URL: `http://192.168.4.1/`

## Hardware Connection
### INMP441 Microphone Wiring

| INMP441 Pin | ESP32 Pin |
|-------------|-----------|
| VDD         | 3V3      |
| GND         | GND      |
| BCLK/SCK    | GPIO26   |
| LRCK/WS     | GPIO25   |
| DOUT/SD     | GPIO32   |
| L/R (SEL)   | 3V3      |

**Note:** L/R (SEL) is set for RIGHT slot. Use hotkeys if your board configuration differs.

## Setup Instructions
Before flashing, configure Arduino IDE:
1. Tools → Partition Scheme
2. Select either:
   - No OTA (2MB APP/2MB SPIFFS)
   - Any partition scheme with ≥1MB SPIFFS