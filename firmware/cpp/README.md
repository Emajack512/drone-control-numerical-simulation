# Drone Firmware (Arduino/C++)

This folder contains the **embedded (Arduino/C++)** code used in the multirotor drone project.
It is organized to clearly separate the **main flight controller** from **sensor tests** and **utilities**.

## Folder structure
- `main/` — **Main flight controller firmware (V4)** (recommended entry point)
- `sensors/` — Sensor bring-up / validation sketches (e.g., IMU tests)
- `utils/` — Utilities and small helper sketches/tools

## Main version
✅ Current main version: **V4**

Open the sketch located in:
- `main/`

> Note: In Arduino IDE, the `.ino` file must be inside a folder with the same name.
> If you get an Arduino warning, ensure the folder/sketch naming matches.

## Build / Upload
- Framework: Arduino
- Target board: (ESP32 / STM32 / other — specify)
- Recommended IDE: Arduino IDE or PlatformIO

### UART / Telemetry (if applicable)
- (Add baudrate and ports if you use serial telemetry)

## What’s not included
- Remote controller firmware/hardware (owned by teammates / separate repository)
- Team-private details not required for a public portfolio

## License
MIT (see repository `LICENSE`).
