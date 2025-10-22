#include <stdio.h>              // Include standard input/output library
#include "pico/stdlib.h"        // Include Pico SDK standard library for GPIO functions

#define RED_LED_PIN 13              // Assign GPIO pin 13 for Red LED
#define GREEN_LED_PIN 14            // Assign GPIO pin 14 for Green LED
#define BLUE_LED_PIN 15             // Assign GPIO pin 15 for Blue LED

int main(void) {
    stdio_init_all();                           // Initialize all standard I/O interfaces
    
    gpio_init(RED_LED_PIN);                     // Initialize Red LED GPIO pin
    gpio_set_dir(RED_LED_PIN, GPIO_OUT);        // Set Red LED pin as output
    gpio_put(RED_LED_PIN, 0);                   // Set Red LED to LOW (off)
    
    gpio_init(GREEN_LED_PIN);                   // Initialize Green LED GPIO pin
    gpio_set_dir(GREEN_LED_PIN, GPIO_OUT);      // Set Green LED pin as output
    gpio_put(GREEN_LED_PIN, 0);                 // Set Green LED to LOW (off)
    
    gpio_init(BLUE_LED_PIN);                    // Initialize Blue LED GPIO pin
    gpio_set_dir(BLUE_LED_PIN, GPIO_OUT);       // Set Blue LED pin as output
    gpio_put(BLUE_LED_PIN, 0);                  // Set Blue LED to LOW (off)
    
    while (1) {                                 // Infinite loop to repeat the 10-second cycle
        // Phase 1: All LEDs OFF for 5 seconds
        gpio_put(RED_LED_PIN, 0);               // Turn off Red LED
        gpio_put(GREEN_LED_PIN, 0);             // Turn off Green LED
        gpio_put(BLUE_LED_PIN, 0);              // Turn off Blue LED
        sleep_ms(5000);                         // Wait for 5000 milliseconds (5 seconds)
        
        // Phase 2: Red LED ON for 1 second
        gpio_put(RED_LED_PIN, 1);               // Turn on Red LED
        sleep_ms(1000);                         // Wait for 1000 milliseconds (1 second)
        gpio_put(RED_LED_PIN, 0);               // Turn off Red LED
        
        // Phase 3: Green LED ON for 1 second
        gpio_put(GREEN_LED_PIN, 1);             // Turn on Green LED
        sleep_ms(1000);                         // Wait for 1000 milliseconds (1 second)
        gpio_put(GREEN_LED_PIN, 0);             // Turn off Green LED
        
        // Phase 4: Blue LED ON for 1 second
        gpio_put(BLUE_LED_PIN, 1);              // Turn on Blue LED
        sleep_ms(1000);                         // Wait for 1000 milliseconds (1 second)
        gpio_put(BLUE_LED_PIN, 0);              // Turn off Blue LED
        
        // Phase 5: All three LEDs ON for 2 seconds
        gpio_put(RED_LED_PIN, 1);               // Turn on Red LED
        gpio_put(GREEN_LED_PIN, 1);             // Turn on Green LED
        gpio_put(BLUE_LED_PIN, 1);              // Turn on Blue LED
        sleep_ms(2000);                         // Wait for 2000 milliseconds (2 seconds)
        
        // Cycle complete - loop repeats (10 seconds total: 5 + 1 + 1 + 1 + 2)
    }
} 
