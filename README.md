# ESP32-Wroom32 and INMP441: Push-to-record, BLE notify stream, HTTP audio download.

A project combining ESP32 with INMP441 I2S microphone featuring BLE GATT (IMA-ADPCM), WAV recording (SPIFFS), and HTTP file browser functionality. The initial idea was to stream recorded audio via BLE but the ESP32 support isn't great. It was added an HTTP fileserver to quickly fetch the recorded audio, which is activated once the recording is stopped. Maintaining the WiFi AP active while recording was not great for the voice quality.

## Features
- BLE GATT with IMA-ADPCM encoding (not really used anymore -- but the ESP would be visible in the BLE devices by your phone)
- WAV recording to SPIFFS
- HTTP file browser
- Push-to-record functionality (hold BOOT button)
- Silence watchdog
- Gain control
- Wi-Fi hotspot controls

## Hardware Wiring

- MCU: ESP32 Wroom 32 (DevKit v1): https://de.aliexpress.com/item/1005001627605230.html
- Microphone: https://de.aliexpress.com/item/33006854069.html 

<img width="512" height="512" alt="Hardware Oct 31, 2025, 10_24_14 AM" src="https://github.com/user-attachments/assets/cf1169da-4a27-44fa-af84-124c856fed5c" />
<img width="512" height="512" alt="Wiring Oct 31, 2025, 10_24_14 AM" src="https://github.com/user-attachments/assets/b276de3f-de0b-482f-a72e-c3cff12ac135" />

**Note:** L/R (SEL) is set for RIGHT slot. Use hotkeys if your board configuration differs. Another hint, do the welding to have a more reliable connection.

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

## Setup Instructions
Before flashing, configure Arduino IDE:
1. Tools → Partition Scheme
2. Select either:
   - No OTA (2MB APP/2MB SPIFFS)
   - Any partition scheme with ≥1MB SPIFFS
  
<img width="470" height="91" alt="image" src="https://github.com/user-attachments/assets/14c1a285-df6a-4a8a-9920-186eb9a4cc8a" />

## Output sample

[rec-020.wav](https://github.com/user-attachments/files/23260313/rec-020.wav)

