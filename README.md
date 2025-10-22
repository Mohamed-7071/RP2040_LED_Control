Embedded2 - RP2040 RGB LED demo

This project demonstrates a 10-second repeating RGB LED sequence for an RP2040 board (Arduino RP2040 / Raspberry Pi Pico compatible).

Build and flash (example):
1. Configure Pico SDK and build with CMake and ninja
2. Run ninja in the `build` folder
3. Copy the generated `Embedded2.uf2` to the Pico when it's in BOOTSEL mode

GPIO pins used:
- GPIO13 - Red
- GPIO14 - Green
- GPIO15 - Blue

License: MIT
