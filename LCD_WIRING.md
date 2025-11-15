# LCD Wiring Guide

## I2C LCD 16x2 Connections

Connect your I2C LCD to the Pico:

| LCD Pin | Pico Pin | Description |
|---------|----------|-------------|
| VCC     | VBUS (pin 40) or 3.3V (pin 36) | Power (5V or 3.3V depending on LCD) |
| GND     | GND (any GND pin) | Ground |
| SDA     | GP0 (pin 1) | I2C Data |
| SCL     | GP1 (pin 2) | I2C Clock |

## Notes

- Most I2C LCD modules have a **PCF8574 I2C backpack** chip
- Common I2C addresses: **0x27** or **0x3F**
- If "Hello" doesn't appear, try changing `LCD_ADDR` to `0x3F` in the code
- The LCD will display "Hello" on startup (top-left corner)

## Current Pin Usage

| Function | Pico Pin |
|----------|----------|
| LCD SDA  | GP0 |
| LCD SCL  | GP1 |
| Motor X IN1 | GP2 |
| Motor X IN2 | GP3 |
| Motor X PWM | GP6 |
| Potentiometer | GP28 (ADC2) |

## Testing

After flashing:
1. LCD should light up with backlight
2. "Hello" appears on the first line
3. Serial monitor still works for motor commands
