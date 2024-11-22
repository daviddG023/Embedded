#include "lcd.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

// Define LCD Pins
#define RS 25
#define E  15
#define D4 16
#define D5 17
#define D6 18
#define D7 19

// Define LCD Commands
#define LCD_LINE_1 0x80 // Move cursor to the first line
#define LCD_LINE_2 0xC0 // Move cursor to the second line

// Helper function to generate enable pulse
void lcd_pulse_enable() {
    gpio_put(E, 1);
    sleep_us(2);  // Extended pulse duration
    gpio_put(E, 0);
    sleep_us(150);  // Increased settle time
}

// Send a 4-bit nibble to the LCD
void lcd_send_nibble(uint8_t nibble) {
    gpio_put(D4, (nibble >> 0) & 1);
    gpio_put(D5, (nibble >> 1) & 1);
    gpio_put(D6, (nibble >> 2) & 1);
    gpio_put(D7, (nibble >> 3) & 1);
    lcd_pulse_enable();
}

// Send a full byte to the LCD
void lcd_send_byte(uint8_t data, bool is_command) {
    gpio_put(RS, is_command ? 0 : 1);
    lcd_send_nibble(data >> 4);  // Send high nibble
    lcd_send_nibble(data & 0x0F); // Send low nibble
    sleep_us(40);
}

// Initialize the LCD
void lcd_init() {
    // Set GPIO pins as output
    gpio_init(RS);
    gpio_init(E);
    gpio_init(D4);
    gpio_init(D5);
    gpio_init(D6);
    gpio_init(D7);

    gpio_set_dir(RS, GPIO_OUT);
    gpio_set_dir(E, GPIO_OUT);
    gpio_set_dir(D4, GPIO_OUT);
    gpio_set_dir(D5, GPIO_OUT);
    gpio_set_dir(D6, GPIO_OUT);
    gpio_set_dir(D7, GPIO_OUT);

    // LCD Initialization Sequence
    sleep_ms(50); // Wait for LCD to power up
    lcd_send_nibble(0x03);
    sleep_ms(50);
    lcd_send_nibble(0x03);
    sleep_us(150);
    lcd_send_nibble(0x03);
    lcd_send_nibble(0x02); // Set 4-bit mode

    // Configure LCD
    lcd_send_byte(0x28, true); // 4-bit, 2 lines, 5x8 dots
    lcd_send_byte(0x0C, true); // Display ON, Cursor OFF
    lcd_send_byte(LCD_CLEAR, true); // Clear display
    lcd_send_byte(0x06, true); // Entry mode: move cursor right
    sleep_ms(20);
}

// Print a string to the LCD (current cursor position)
void lcd_print(const char *str) {
    while (*str) {
        lcd_send_byte(*str++, false);
    }
}

// Print a string to the LCD on the second line
void lcd_print2(const char *str) {
    lcd_send_byte(LCD_LINE_2, true); // Move cursor to the second line
    while (*str) {
        lcd_send_byte(*str++, false);
    }
}
