# LCD Wiring (No Soldering - Using Jumpers)

## What you need:
- I2C LCD 16x2 module
- 4 female-to-female jumper wires
- Breadboard (optional)

## Connections:

| LCD Pin | Wire Color (suggestion) | Pico Pin | Pin Number |
|---------|------------------------|----------|------------|
| VCC     | Red                    | VBUS     | Pin 40     |
| GND     | Black                  | GND      | Pin 38     |
| SDA     | Blue/Green             | GP8      | Pin 11     |
| SCL     | Yellow/White           | GP7      | Pin 10     |

## Notes:
- **No soldering needed** - just plug jumper wires into the LCD module pins
- If LCD doesn't light up, try changing `LCD_ADDR` from `0x27` to `0x3F` in the code
- The LCD backlight will turn on when powered

## What happens:
1. LED on GP2 blinks 5 times
2. LCD displays "Hello" 
3. After 5 seconds, LCD clears
4. Program ends

## If LCD doesn't work:
Try changing the address in line 13 of Embedded2.c:
```c
#define LCD_ADDR 0x27  // Change to 0x3F if 0x27 doesn't work
```
