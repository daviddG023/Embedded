#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "drivers/lcdDriver/lcd.h"
#include "drivers/rfidDriver/rfid.h"


#include "drivers/buzzerDriver/buzzer.h"
#include "drivers/servoDriver/servo.h"
#include "drivers/irDriver/ir_sensor.h"
#include "drivers/lcdDriver/lcd.h"
#include "drivers/ultrasonicDriver/ultrasonic.h"
#include "../include/hardware/gpio.h"
#include "pwm.h"
#include <string.h>

// Define the GPIO pins for the peripherals
#define BUZZER_PIN 26      // Buzzer GPIO pin
#define SERVO_PIN 20       // Servo GPIO pin
#define IR_SENSOR_ADC_PIN 27 // IR sensor ADC pin
#define TRIG_PIN 29      // Ultrasonic sensor TRIG pin
#define ECHO_PIN 28         // Ultrasonic sensor ECHO pin

// RFID Pin Definitions
#define SDA_PIN 21 // D9
#define SCK_PIN 6  // D13
#define MOSI_PIN 7 // D11
#define MISO_PIN 4 // D12
#define RST_PIN 5  // D10



void setup_all(MFRC522Ptr_t *mfrc) {
    // Initialize stdio for serial communication
    stdio_init_all();

    // Initialize the LCD
    lcd_init();

    // Initialize the RFID reader
    rfid_init(mfrc);
    buzzer_init(BUZZER_PIN);
    servo_init(SERVO_PIN);
    ir_sensor_init_adc(IR_SENSOR_ADC_PIN);

    hc_sr04_init(TRIG_PIN, ECHO_PIN);
    // Display a ready message on the LCD
    lcd_print("System Ready!");
    sleep_ms(2000);
    lcd_send_byte(LCD_CLEAR, true);
}

int main() {
    // Declare an MFRC522 pointer
    MFRC522Ptr_t mfrc;

    // Pass the address of mfrc to the setup function
    setup_all(&mfrc);
    


    uint8_t uid[10];
    uint8_t uid_length;
    char rfid_message[32];
    printf("Unified System Initialized\n");

    while (true) {
    lcd_send_byte(LCD_CLEAR, true); // Clear LCD screen
    sleep_ms(1000);

    // Check if a card is present
    if (card_present(mfrc)) {
        if (read_card_uid(mfrc, uid, &uid_length)) {
            // Format the UID as a string for display
            char uid_str[30];
            sprintf(uid_str, "UID: ");
            for (uint8_t i = 0; i < uid_length; i++) {
                sprintf(uid_str + strlen(uid_str), "%02X ", uid[i]); // Append each byte in hex
            }
            lcd_print(uid_str); // Print the UID on the LCD
            sleep_ms(1000);
            // // **3. Rotate Servo Motor**
            printf("Rotating Servo...\n");
            uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
            rotate_servo(slice_num);

            // **5. Simple Buzzer ON/OFF**
            printf("Playing 1kHz tone for 500ms\n");
            buzzer_play_tone(BUZZER_PIN, 1000, 500); // 1kHz tone for 500ms
            sleep_ms(500);                           // Wait for 500ms

            // Play a 500Hz tone for 1 second
            printf("Playing 500Hz tone for 1 second\n");
            buzzer_play_tone(BUZZER_PIN, 500, 1000); // 500Hz tone for 1 second
            sleep_ms(500);                           // Wait for 500ms

            // Play a 2kHz tone for 250ms
            printf("Playing 2kHz tone for 250ms\n");
            buzzer_play_tone(BUZZER_PIN, 2000, 250); // 2kHz tone for 250ms

            sleep_ms(5000);

        }
    } else {
        lcd_print("No Card Present");
        sleep_ms(2000);
    }

// Read and Display IR Sensor Data
    uint16_t ir_raw_value = ir_sensor_read_adc();
    float ir_voltage = ir_sensor_get_voltage(ir_raw_value);

    char ir_message[32];
    if (ir_voltage > 3.3 || ir_voltage < 0) {
        strcpy(ir_message, "Invalid IR");
    } else {
        snprintf(ir_message, sizeof(ir_message), "IR: %.2fV", ir_voltage);
    }

    // Clear the LCD and reset cursor
    lcd_send_byte(LCD_CLEAR, true); // Clear the LCD
    sleep_ms(10); // Add a delay
    lcd_send_byte(0x80, true); // Reset cursor to the first line

    // Display the message
    lcd_print(ir_message);
    printf("IR Sensor: %.2fV\n", ir_voltage);


    // ___________________________________________________________________________________________________________________

    // **2. Measure and Display Ultrasonic Sensor Distance**
    float distance = measure_distance(TRIG_PIN, ECHO_PIN);
    char us_message[32];
    if (distance < 0) {
        sprintf(us_message, "No Obj Detected");
        printf("Ultrasonic: Timeout! No object detected.\n");
    } else {
        sprintf(us_message, "Distance: %.2f cm", distance);
        printf("Ultrasonic: %.2f cm\n", distance);
    }
    lcd_print2(us_message);
    sleep_ms(2000);
    lcd_send_byte(LCD_CLEAR, true);
    sleep_ms(2000); // Delay before next check
}


    return 0;
}
