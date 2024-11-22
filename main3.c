#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "drivers/lcdDriver/lcd.h"
#include "mfrc522.h"

// RFID Pin Definitions
#define SDA_PIN 21 // D9
#define SCK_PIN 6  // D13
#define MOSI_PIN 7 // D11
#define MISO_PIN 4 // D12
#define RST_PIN 5  // D10




// Predefined Card UID for Authentication
uint8_t authorized_uid[] = {0x93, 0xE3, 0x9A, 0x92}; // Replace with your target card UID

void setup_all() {
    // Initialize stdio for serial communication
    stdio_init_all();

    // Initialize the LCD
    lcd_init();


    // Display ready message on the LCD
    lcd_print("System Ready!");
    sleep_ms(2000);
    lcd_send_byte(LCD_CLEAR, true);
}

int main() {
    setup_all();

    MFRC522Ptr_t mfrc = MFRC522_Init();
    PCD_Init(mfrc, spi0);

    uint8_t uid[10];
    uint8_t uid_length;

    while (true) {
    lcd_send_byte(LCD_CLEAR, true); // Clear LCD screen
    sleep_ms(1000);

    // Check if a card is present
    if (PICC_IsNewCardPresent(mfrc)) {
        // Read card UID
        lcd_print("Access Granted!");
        PICC_HaltA(mfrc); // Halt the card to prepare for the next one
    } else {
        lcd_print("No Card Present");
    }

    sleep_ms(2000); // Delay before next check
}


    return 0;
}
