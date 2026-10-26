# Mindburner MALICE — Firmware

MALICE is a 6-voice polyphonic / monophonic virtual analogue MIDI synthesiser module with a built-in step sequencer, built around the ESP32. This repo hosts the firmware and a one-click browser-based installer — no Arduino IDE required.

## Install or update firmware

👉 **[Open the installer](https://mindburners.github.io/MALICE/)**

1. Connect MALICE to your computer with a USB **data** cable
2. Open the link above in **Chrome**, **Edge**, or **Opera**
3. Click **Connect**, choose the correct serial port, then click **Install**

Updating keeps your saved patches, sequencer patterns, MIDI channel and tuning.

> ⚠️ Not supported in Safari or Firefox (no Web Serial support).

## Documentation

- [User manual (PDF)](MALICE_User_Manual.pdf)
- [Quick start guide (PDF)](MALICE_Quick_Start.pdf)

## What's in this repo

| File | Purpose |
|---|---|
| `index.html` | The installer page |
| `changelog.txt` | Version history, newest first. The page reads its version and What's New list from here |
| `manifest.json` | Backup installer manifest (the page builds its own from `changelog.txt`) |
| `bootloader.bin`, `partitions.bin`, `firmware.bin` | The compiled firmware |
| `logo.png` | Branding shown on the installer page |
| `*.pdf` | User manual and quick start guide |

## Releasing an update

1. Upload the new sketch into the `sketch` folder (delete the old .ino so there is only one)
2. Add a new line at the **top** of `changelog.txt`, e.g. `1.5 - what changed`
3. Commit

GitHub compiles the sketch (ESP32 core 2.0.17, ESP32 Dev Module) and republishes the page in about 3-5 minutes.
Watch it on the **Actions** tab: a green tick means the new firmware is live. A red cross means the build failed
and the page keeps the previous version - click it to see the error.

New manuals: upload them with the same file names and commit. The page republishes automatically.

## Hardware

- ESP32-WROOM-32E
- PCM5102A stereo I2S DAC
- ST7735 160×128 SPI colour TFT
- 4 buttons and rotary encoder, DIN MIDI in / thru

---
Mindburner — Alan G. Watkins
