# PCBOven

PCBOven is a compact ESP32-based oven controller designed to automate heating and cooling cycles for PCB reflow/soldering. The project provides:

- Thermocouple temperature reading via the Adafruit MAX31856
- SSR (PWM) control for the heater element
- PWM-driven fan control for cooling
- A simple web UI (served from SPIFFS) with WebSocket telemetry and an HTTP toggle endpoint

Purpose
 - Provide a simple, WiFi-accessible interface to heat a PCB to a target temperature and run for a configurable duration.

Hardware requirements
- ESP32 development board (example: esp32doit-devkit-v1)
- Adafruit MAX31856 thermocouple breakout (J-type configured in code)
- Thermocouple (J-type)
- SSR for heater control (drive via PWM-capable pin)
- PWM-capable fan (or MOSFET driver) for cooling
- Appropriate power supply for heater and fan (not from ESP32 3.3V)

Wiring (high-level)
- MAX31856 CS -> pin defined by `MAX_CS` (default 5)
- MAX31856 SPI pins (SCK/MOSI/MISO) -> standard ESP32 SPI pins
- SSR signal -> pin defined by `SSR` (default 13)
- Fan PWM -> pin 27 (as used in code)

Configuration
- Update WiFi credentials in `src/Web_server_Ovenv3.cpp` before flashing.
- Verify pin assignments match your hardware. The code uses `LED_PIN` (2) for status LED, `SSR` (13) for heater PWM and pin 27 for fan PWM.

Building and flashing
1. Install PlatformIO in VS Code or use the PlatformIO CLI.
2. From the project root run:

```bash
platformio run --environment esp32doit-devkit-v1 --target upload
```

3. Open the serial monitor to observe logs:

```bash
platformio device monitor --environment esp32doit-devkit-v1
```

Usage
- After boot the device connects to WiFi and serves the web UI (index.html) from SPIFFS.
- Use the web UI to set `Desired Heat Temperature` and `Maximum Heating Time` (minutes) and toggle the heating function.
- The UI receives telemetry via WebSocket at `/ws` and posts parameters to `/toggle`.

Notes, known issues and suggestions
- The project uses simple heuristics for heater/fan control, not a full PID — expect manual tuning for reliable reflow profiles.
- Check the `ledcAttach` usage and PWM channel mapping for your ESP32 core. Some cores use `ledcSetup`/`ledcAttachPin` APIs.
- Ensure the SSR and fan power supply are sized correctly for the heater load; do not power heaters from the ESP32.
- The WiFi credentials are hard-coded; consider using a safer provisioning method for production.
