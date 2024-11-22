#ifndef LCD_H
#define LCD_H

#include <stdint.h>
#include <stdbool.h>

// Define LCD Commands
#define LCD_CLEAR 0x01
#define LCD_HOME  0x02


// Function Prototypes
void lcd_init();
void lcd_print(const char *str);
void lcd_send_byte(uint8_t data, bool is_command);
void lcd_send_nibble(uint8_t nibble);
void lcd_print2(const char *str);

#endif // LCD_H
