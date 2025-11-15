// Simple LED Blink + LCD "Hello" Test for RP2040

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"

// ===== LCD I2C CONFIGURATION =====
#define I2C_PORT i2c0
#define I2C_SDA 0   // GP0 - SDA (Physical Pin 1)
#define I2C_SCL 1   // GP1 - SCL (Physical Pin 2)
#define LCD_ADDR 0x27  // Try 0x27 first, then 0x3F if doesn't work

// LCD Commands
#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_FUNCTIONSET 0x20
#define LCD_SETDDRAMADDR 0x80

// LCD Flags
#define LCD_DISPLAYON 0x04
#define LCD_CURSOROFF 0x00
#define LCD_BLINKOFF 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTDECREMENT 0x00
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_5x8DOTS 0x00
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

#define En 0b00000100  // Enable bit
#define Rw 0b00000010  // Read/Write bit
#define Rs 0b00000001  // Register select bit

// ===== GPIO PIN DEFINITIONS =====
#define LED_PIN         2   // GP2 → LED (blinks 5 times on startup)

// Function prototypes
void lcd_init(void);
void lcd_write_nibble(uint8_t nibble);
void lcd_toggle_enable(uint8_t val);
void lcd_send_byte(uint8_t val, int mode);
void lcd_clear(void);
void lcd_set_cursor(int line, int position);
void lcd_string(const char *s);
void i2c_scan(void);

// Motor definitions
#define MOTOR_X_DIR_1   18   // GP18 → L298N IN3
#define MOTOR_X_DIR_2   17   // GP17 → L298N IN4
#define MOTOR_X_PWM     16   // GP16 → L298N ENa (PWM)
#define POT_PIN         26   // GP26 → Potentiometer (ADC0)

// PWM configuration
#define PWM_FREQ_HZ     20000u
#define PWM_WRAP        6249    // For 20kHz at 125MHz: (125000000/20000)-1

// Motor function prototypes
void setup_motor(void);
void motorX_CW(void);
void motorX_CCW(void);
void stopAll(void);
void set_motor_pwm(uint8_t duty_percent);
void test_potentiometer(void);
void potentiometer_control_motor(void);

/*
#define MOTOR_X_DIR_1   2   // GP2 → L298N IN1
#define MOTOR_X_DIR_2   3   // GP3 → L298N IN2
// #define MOTOR_Y_DIR_1   4   // GP4 → L298N IN3
// #define MOTOR_Y_DIR_2   5   // GP5 → L298N IN4
#define MOTOR_X_PWM     6   // GP6 → L298N ENa (PWM)
#define POT_X           28  // GP28 → Potentiometer X (ADC2)

// PWM configuration
#define PWM_FREQ_HZ     20000u
#define PWM_WRAP        6249    // For 20kHz at 125MHz: (125000000/20000)-1

// Function prototypes
void setup_hardware(void);
void motorX_CW(void);
void motorX_CCW(void);
// void motorY_CW(void);
// void motorY_CCW(void);
void stopAll(void);
void potentiometerControl(void);
void set_motor_pwm(uint8_t duty_percent);

// Helper function to map values
int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
*/

int main(void) {
    stdio_init_all();
    sleep_ms(2000); // Give serial monitor time to connect
    
    printf("\n========== RP2040 LCD TEST ==========\n");
    
    // Initialize LED pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
    
    // Blink LED 5 times
    printf("Blinking LED...\n");
    for (int i = 0; i < 5; i++) {
        gpio_put(LED_PIN, 1);  // LED ON
        sleep_ms(500);
        gpio_put(LED_PIN, 0);  // LED OFF
        sleep_ms(500);
    }
    
    // Initialize I2C for LCD
    printf("Initializing I2C on GP%d (SDA) and GP%d (SCL)...\n", I2C_SDA, I2C_SCL);
    i2c_init(I2C_PORT, 100 * 1000); // 100 kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    sleep_ms(100);
    
    // Scan for I2C devices
    printf("\nScanning I2C bus...\n");
    i2c_scan();
    
    // Initialize LCD
    printf("\nInitializing LCD at address 0x%02X...\n", LCD_ADDR);
    lcd_init();
    printf("LCD initialized!\n");
    
    // Display "Hello" on LCD
    lcd_clear();
    lcd_set_cursor(0, 0);  // Line 0, position 0
    lcd_string("3 in 1 Special");
    lcd_set_cursor(1, 0);  // Line 1, position 0
    lcd_string("RP2040 LCD OK");
    
    printf("\nText sent to LCD. Check display!\n");
    
    // Initialize motor
    printf("\nInitializing motor...\n");
    setup_motor();
    
    // Initialize ADC for potentiometer
    printf("Initializing ADC for potentiometer on GP%d...\n", POT_PIN);
    adc_init();
    adc_gpio_init(POT_PIN);
    adc_select_input(0); // ADC0 for GP26
    
    // Test potentiometer for 10 seconds
    printf("\n========== POTENTIOMETER TEST ==========\n");
    printf("Rotate the potentiometer and watch the values...\n");
    printf("Testing for 10 seconds...\n\n");
    test_potentiometer();
    
    sleep_ms(2000);
    
    // Potentiometer control mode - replaces fixed motor tests
    printf("\n========== POTENTIOMETER MOTOR CONTROL ==========\n");
    printf("Rotate potentiometer to control motor speed and direction\n");
    printf("Left = CCW | Center = STOP | Right = CW\n");
    printf("Running for 30 seconds...\n\n");
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_string("POT Control ON");
    sleep_ms(2000);
    
    potentiometer_control_motor();
    
    // Stop motor after control
    stopAll();
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_string("Motor: STOPPED");
    
    // Keep program running
    while (1) {
        tight_loop_contents();
    }
}

/*
// === OLD CODE - COMMENTED OUT ===
/*
int main_old(void) {
    stdio_init_all();
    setup_hardware();
    
    // Blink LED 5 times on startup
    for (int i = 0; i < 5; i++) {
        gpio_put(LED_PIN, 1);  // LED ON
        sleep_ms(200);
        gpio_put(LED_PIN, 0);  // LED OFF
        sleep_ms(200);
    }
    
    sleep_ms(1000);
    
    // Display "Hello" on LCD
    lcd_clear();
    lcd_set_cursor(0, 0);  // Line 0, position 0
    lcd_string("Hello");
    
    printf("\n========== MOTOR TEST START ==========\n");
    printf("Commands:\n");
    printf("  1 - Motor X CW (Clockwise) - full speed\n");
    printf("  2 - Motor X CCW (Counter-Clockwise) - full speed\n");
    // printf("  3 - Motor Y CW (Clockwise) - full speed\n");
    // printf("  4 - Motor Y CCW (Counter-Clockwise) - full speed\n");
    printf("  5 - Stop all motors\n");
    printf("  6 - Potentiometer control (real-time)\n");
    printf("========================================\n\n");
    
    while (1) {
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            switch (c) {
                case '1':
                    motorX_CW();
                    break;
                case '2':
                    motorX_CCW();
                    break;
                // case '3':
                //     motorY_CW();
                //     break;
                // case '4':
                //     motorY_CCW();
                //     break;
                case '5':
                    stopAll();
                    break;
                case '6':
                    potentiometerControl();
                    break;
            }
        }
        sleep_ms(10);
    }
}

void setup_hardware(void) {
    // Initialize LED pin
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
    
    // Initialize I2C for LCD
    i2c_init(I2C_PORT, 100 * 1000); // 100 kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    // Initialize LCD
    lcd_init();
    
    // Initialize direction pins
    gpio_init(MOTOR_X_DIR_1);
    gpio_set_dir(MOTOR_X_DIR_1, GPIO_OUT);
    gpio_put(MOTOR_X_DIR_1, 0);
    
    gpio_init(MOTOR_X_DIR_2);
    gpio_set_dir(MOTOR_X_DIR_2, GPIO_OUT);
    gpio_put(MOTOR_X_DIR_2, 0);
    
    //gpio_init(MOTOR_Y_DIR_1);
    //gpio_set_dir(MOTOR_Y_DIR_1, GPIO_OUT);
    //gpio_put(MOTOR_Y_DIR_1, 0);
    
    //gpio_init(MOTOR_Y_DIR_2);
    //gpio_set_dir(MOTOR_Y_DIR_2, GPIO_OUT);
    //gpio_put(MOTOR_Y_DIR_2, 0);
    
    // Initialize PWM for motor speed control
    gpio_set_function(MOTOR_X_PWM, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(MOTOR_X_PWM);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 1.0f);
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(MOTOR_X_PWM, 0);
    
    // Initialize ADC for potentiometer (GP28 = ADC2)
    adc_init();
    adc_gpio_init(POT_X);
}

void set_motor_pwm(uint8_t duty_percent) {
    uint16_t level = (PWM_WRAP * duty_percent) / 100;
    pwm_set_gpio_level(MOTOR_X_PWM, level);
}

// ===== MOTOR X - CLOCKWISE =====
void motorX_CW(void) {
    printf("[TEST] Motor X - CW (full speed) for 3 seconds...\n");
    gpio_put(MOTOR_X_DIR_1, 1);   // IN1 = HIGH
    gpio_put(MOTOR_X_DIR_2, 0);   // IN2 = LOW
    set_motor_pwm(100);           // Full speed
    
    sleep_ms(3000);
    
    set_motor_pwm(0);
    printf("[TEST] Motor X - STOPPED\n\n");
}

// ===== MOTOR X - COUNTER-CLOCKWISE =====
void motorX_CCW(void) {
    printf("[TEST] Motor X - CCW (full speed) for 3 seconds...\n");
    gpio_put(MOTOR_X_DIR_1, 0);   // IN1 = LOW
    gpio_put(MOTOR_X_DIR_2, 1);   // IN2 = HIGH
    set_motor_pwm(100);           // Full speed
    
    sleep_ms(3000);
    
    set_motor_pwm(0);
    printf("[TEST] Motor X - STOPPED\n\n");
}

// ===== MOTOR Y - CLOCKWISE =====
//void motorY_CW(void) {
    //printf("[TEST] Motor Y - CW (full speed) for 3 seconds...\n");
    //gpio_put(MOTOR_Y_DIR_1, 1);   // IN3 = HIGH
    //gpio_put(MOTOR_Y_DIR_2, 0);   // IN4 = LOW
    
    //sleep_ms(3000);
    
    //printf("[TEST] Motor Y - STOPPED\n\n");
//}

// ===== MOTOR Y - COUNTER-CLOCKWISE =====
//void motorY_CCW(void) {
    //printf("[TEST] Motor Y - CCW (full speed) for 3 seconds...\n");
    //gpio_put(MOTOR_Y_DIR_1, 0);   // IN3 = LOW
    //gpio_put(MOTOR_Y_DIR_2, 1);   // IN4 = HIGH

    //sleep_ms(3000);

    //printf("[TEST] Motor Y - STOPPED\n\n");
//}

// ===== STOP ALL MOTORS =====
void stopAll(void) {
    set_motor_pwm(0);
    gpio_put(MOTOR_X_DIR_1, 0);
    gpio_put(MOTOR_X_DIR_2, 0);
    //gpio_put(MOTOR_Y_DIR_1, 0);
    //gpio_put(MOTOR_Y_DIR_2, 0);
    printf("[TEST] All motors STOPPED\n\n");
}

// ===== POTENTIOMETER CONTROL =====
void potentiometerControl(void) {
    printf("[TEST] Potentiometer control active for 30 seconds...\n");
    printf("Rotate potentiometer to control Motor X speed\n\n");
    
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_print = 0;
    
    adc_select_input(2); // ADC2 for GP28
    
    while ((to_ms_since_boot(get_absolute_time()) - start_time) < 30000) {
        uint16_t potX = adc_read();  // Read 0-4095 (12-bit)
        
        // Convert to motor speed (0-100%)
        int speed = map(potX, 0, 4095, 0, 100);
        
        // Determine direction based on potentiometer position
        if (potX < 1600) {
            // CCW - Reverse
            gpio_put(MOTOR_X_DIR_1, 0);
            gpio_put(MOTOR_X_DIR_2, 1);
            set_motor_pwm(100 - speed);
        } else if (potX > 2500) {
            // CW - Forward
            gpio_put(MOTOR_X_DIR_1, 1);
            gpio_put(MOTOR_X_DIR_2, 0);
            set_motor_pwm(speed);
        } else {
            // Stop - middle zone
            set_motor_pwm(0);
        }
        
        // Print every 500ms
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_print >= 500) {
            printf("Pot: %d | Speed: %d%% | Direction: ", potX, speed);
            if (potX < 1600) printf("CCW\n");
            else if (potX > 2500) printf("CW\n");
            else printf("STOP\n");
            
            last_print = now;
        }
        
        sleep_ms(50);
    }
    
    stopAll();
    printf("[TEST] Potentiometer control COMPLETE\n\n");
}
*/

// ===== MOTOR CONTROL FUNCTIONS =====
void setup_motor(void) {
    printf("Setting up motor pins...\n");
    
    // Initialize direction pins
    gpio_init(MOTOR_X_DIR_1);
    gpio_set_dir(MOTOR_X_DIR_1, GPIO_OUT);
    gpio_put(MOTOR_X_DIR_1, 0);
    
    gpio_init(MOTOR_X_DIR_2);
    gpio_set_dir(MOTOR_X_DIR_2, GPIO_OUT);
    gpio_put(MOTOR_X_DIR_2, 0);
    
    // Initialize PWM for motor speed control
    gpio_set_function(MOTOR_X_PWM, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(MOTOR_X_PWM);
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, 1.0f);
    pwm_config_set_wrap(&cfg, PWM_WRAP);
    pwm_init(slice, &cfg, true);
    pwm_set_gpio_level(MOTOR_X_PWM, 0);
    
    printf("Motor initialized on GP%d (DIR1), GP%d (DIR2), GP%d (PWM)\n", 
           MOTOR_X_DIR_1, MOTOR_X_DIR_2, MOTOR_X_PWM);
}

void set_motor_pwm(uint8_t duty_percent) {
    uint16_t level = (PWM_WRAP * duty_percent) / 100;
    pwm_set_gpio_level(MOTOR_X_PWM, level);
}

void motorX_CW(void) {
    gpio_put(MOTOR_X_DIR_1, 1);   // IN1 = HIGH
    gpio_put(MOTOR_X_DIR_2, 0);   // IN2 = LOW
    set_motor_pwm(100);           // Full speed
    sleep_ms(3000);
    set_motor_pwm(0);
}

void motorX_CCW(void) {
    gpio_put(MOTOR_X_DIR_1, 0);   // IN1 = LOW
    gpio_put(MOTOR_X_DIR_2, 1);   // IN2 = HIGH
    set_motor_pwm(100);           // Full speed
    sleep_ms(3000);
    set_motor_pwm(0);
}

void stopAll(void) {
    set_motor_pwm(0);
    gpio_put(MOTOR_X_DIR_1, 0);
    gpio_put(MOTOR_X_DIR_2, 0);
}

void test_potentiometer(void) {
    char buffer[20];
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    
    while ((to_ms_since_boot(get_absolute_time()) - start_time) < 10000) {
        // Read potentiometer value (0-4095 for 12-bit ADC)
        uint16_t pot_value = adc_read();
        
        // Convert to percentage (0-100%)
        int percentage = (pot_value * 100) / 4095;
        
        // Convert to voltage (assuming 3.3V reference)
        float voltage = (pot_value * 3.3f) / 4095.0f;
        
        // Display on serial monitor
        printf("ADC: %4d | Percent: %3d%% | Voltage: %.2fV\n", 
               pot_value, percentage, voltage);
        
        // Display on LCD
        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_string("POT Test:");
        
        lcd_set_cursor(1, 0);
        snprintf(buffer, sizeof(buffer), "Val:%4d %3d%%", pot_value, percentage);
        lcd_string(buffer);
        
        sleep_ms(200); // Update 5 times per second
    }
    
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_string("POT Test Done!");
    printf("\n========== POTENTIOMETER TEST COMPLETE ==========\n\n");
    sleep_ms(2000);
}

void potentiometer_control_motor(void) {
    char buffer[20];
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    uint32_t last_print = 0;
    uint32_t last_lcd_update = 0;
    int last_state = -1; // Track state changes: 0=CCW, 1=STOP, 2=CW
    
    while ((to_ms_since_boot(get_absolute_time()) - start_time) < 30000) {
        // Read potentiometer value (0-4095)
        uint16_t pot_value = adc_read();
        int current_state;
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // Determine direction based on potentiometer position
        if (pot_value < 1365) {
            // Left third: CCW (Counter-Clockwise)
            current_state = 0;
            gpio_put(MOTOR_X_DIR_1, 0);
            gpio_put(MOTOR_X_DIR_2, 1);
            int ccw_speed = 100 - ((pot_value * 100) / 1365);
            set_motor_pwm(ccw_speed);
            
            // Update LCD only every 300ms or on state change
            if (now - last_lcd_update >= 300 || last_state != current_state) {
                lcd_clear();
                lcd_set_cursor(0, 0);
                lcd_string("Motor: CCW");
                lcd_set_cursor(1, 0);
                snprintf(buffer, sizeof(buffer), "POT:%4d Spd:%3d%%", pot_value, ccw_speed);
                lcd_string(buffer);
                last_lcd_update = now;
            }
            
            // Print to serial
            if (now - last_print >= 500) {
                printf("POT: %4d | Direction: CCW | Speed: %3d%%\n", pot_value, ccw_speed);
                last_print = now;
            }
            
        } else if (pot_value > 2730) {
            // Right third: CW (Clockwise)
            current_state = 2;
            gpio_put(MOTOR_X_DIR_1, 1);
            gpio_put(MOTOR_X_DIR_2, 0);
            int cw_speed = ((pot_value - 2730) * 100) / (4095 - 2730);
            set_motor_pwm(cw_speed);
            
            // Update LCD only every 300ms or on state change
            if (now - last_lcd_update >= 300 || last_state != current_state) {
                lcd_clear();
                lcd_set_cursor(0, 0);
                lcd_string("Motor: CW");
                lcd_set_cursor(1, 0);
                snprintf(buffer, sizeof(buffer), "POT:%4d Spd:%3d%%", pot_value, cw_speed);
                lcd_string(buffer);
                last_lcd_update = now;
            }
            
            // Print to serial
            if (now - last_print >= 500) {
                printf("POT: %4d | Direction: CW  | Speed: %3d%%\n", pot_value, cw_speed);
                last_print = now;
            }
            
        } else {
            // Middle third: STOP - Turn off both direction pins AND PWM
            current_state = 1;
            gpio_put(MOTOR_X_DIR_1, 0);
            gpio_put(MOTOR_X_DIR_2, 0);
            set_motor_pwm(0);
            
            // Update LCD only every 300ms or on state change
            if (now - last_lcd_update >= 300 || last_state != current_state) {
                lcd_clear();
                lcd_set_cursor(0, 0);
                lcd_string("Motor: STOPPED");
                lcd_set_cursor(1, 0);
                snprintf(buffer, sizeof(buffer), "POT: %4d", pot_value);
                lcd_string(buffer);
                last_lcd_update = now;
            }
            
            // Print to serial
            if (now - last_print >= 500) {
                printf("POT: %4d | Direction: STOP\n", pot_value);
                last_print = now;
            }
        }
        
        last_state = current_state;
        sleep_ms(50); // Update 20 times per second
    }
    
    printf("\n========== POTENTIOMETER CONTROL COMPLETE ==========\n\n");
}

// ===== LCD I2C FUNCTIONS =====
void lcd_write_nibble(uint8_t nibble) {
    uint8_t data = nibble | LCD_BACKLIGHT;
    i2c_write_blocking(I2C_PORT, LCD_ADDR, &data, 1, false);
}

void lcd_toggle_enable(uint8_t val) {
    sleep_us(1000);
    lcd_write_nibble(val | En);
    sleep_us(1000);
    lcd_write_nibble(val & ~En);
    sleep_us(1000);
}

void lcd_send_byte(uint8_t val, int mode) {
    uint8_t high = mode | (val & 0xF0) | LCD_BACKLIGHT;
    uint8_t low = mode | ((val << 4) & 0xF0) | LCD_BACKLIGHT;

    lcd_write_nibble(high);
    lcd_toggle_enable(high);
    lcd_write_nibble(low);
    lcd_toggle_enable(low);
}

void lcd_clear(void) {
    lcd_send_byte(LCD_CLEARDISPLAY, 0);
    sleep_ms(2);
}

void lcd_set_cursor(int line, int position) {
    int val = (line == 0) ? 0x80 + position : 0xC0 + position;
    lcd_send_byte(val, 0);
}

void lcd_string(const char *s) {
    while (*s) {
        lcd_send_byte(*s++, Rs);
    }
}

void lcd_init(void) {
    // Wait for LCD to power up (critical for some displays)
    sleep_ms(100);
    
    // Initialize in 4-bit mode with proper timing
    // Step 1: Send 0x03 three times
    lcd_write_nibble(0x30 | LCD_BACKLIGHT);
    lcd_toggle_enable(0x30 | LCD_BACKLIGHT);
    sleep_ms(5);
    
    lcd_write_nibble(0x30 | LCD_BACKLIGHT);
    lcd_toggle_enable(0x30 | LCD_BACKLIGHT);
    sleep_ms(1);
    
    lcd_write_nibble(0x30 | LCD_BACKLIGHT);
    lcd_toggle_enable(0x30 | LCD_BACKLIGHT);
    sleep_ms(1);
    
    // Step 2: Switch to 4-bit mode
    lcd_write_nibble(0x20 | LCD_BACKLIGHT);
    lcd_toggle_enable(0x20 | LCD_BACKLIGHT);
    sleep_ms(1);
    
    // Function set: 4-bit, 2 lines, 5x8 font
    lcd_send_byte(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS, 0);
    sleep_ms(1);
    
    // Display control: display on, cursor off, blink off
    lcd_send_byte(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF, 0);
    sleep_ms(1);
    
    // Clear display
    lcd_clear();
    sleep_ms(2);
    
    // Entry mode: left to right, no shift
    lcd_send_byte(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT, 0);
    sleep_ms(1);
    
    // Extra delay to ensure LCD is ready
    sleep_ms(10);
}

// ===== I2C SCANNER =====
void i2c_scan(void) {
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\\n");
    
    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }
        
        // Perform a 1-byte dummy read from the probe address
        uint8_t rxdata;
        int ret = i2c_read_blocking(I2C_PORT, addr, &rxdata, 1, false);
        
        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\\n" : "  ");
    }
    printf("\\nDone. '@' = device found, '.' = no device\\n");
    printf("If you see a device at 0x27 or 0x3F, that's likely your LCD!\\n");
}
