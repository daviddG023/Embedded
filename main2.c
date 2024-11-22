#include <stdio.h>
#include "pico/stdlib.h"
#include "drivers/buzzerDriver/buzzer.h"
#include "drivers/servoDriver/servo.h"
#include "drivers/irDriver/ir_sensor.h"
#include "drivers/lcdDriver/lcd.h"
#include "drivers/ultrasonicDriver/ultrasonic.h"
#include "../include/hardware/gpio.h"
#include "pwm.h"
#include <string.h>

#define BUZZER_PIN 26      // Buzzer GPIO pin
#define SERVO_PIN 20       // Servo GPIO pin
#define IR_SENSOR_ADC_PIN 27 // IR sensor ADC pin
#define TRIG_PIN 29      // Ultrasonic sensor TRIG pin
#define ECHO_PIN 28         // Ultrasonic sensor ECHO pin

// Define the GPIO pins for the RFID reader
#define SDA_PIN 21 // D9
#define SCK_PIN 6  // D13
#define MOSI_PIN 7 // D11
#define MISO_PIN 4 // D12
#define RST_PIN 5  // D10

void setup_all() {
    // Initialize all peripherals
    stdio_init_all();
    buzzer_init(BUZZER_PIN);
    servo_init(SERVO_PIN);
    ir_sensor_init_adc(IR_SENSOR_ADC_PIN);
    lcd_init();
    hc_sr04_init(TRIG_PIN, ECHO_PIN);


    // // Display initialization message on the LCD
    lcd_print("System Ready!");
    sleep_ms(2000); // Pause to show the message
    lcd_send_byte(LCD_CLEAR, true); // Clear the LCD for further use
    sleep_ms(2000); // Pause to show the message
}

int main() {
    setup_all();
       
    char rfid_message[32];
    printf("Unified System Initialized\n");

    while (true) {
        // // Read and Display IR Sensor Data
        // uint16_t ir_raw_value = ir_sensor_read_adc();
        // float ir_voltage = ir_sensor_get_voltage(ir_raw_value);

        // char ir_message[32];
        // if (ir_voltage > 3.3 || ir_voltage < 0) {
        //     strcpy(ir_message, "Invalid IR");
        // } else {
        //     snprintf(ir_message, sizeof(ir_message), "IR: %.2fV", ir_voltage);
        // }

        // // Clear the LCD and reset cursor
        // lcd_send_byte(LCD_CLEAR, true); // Clear the LCD
        // sleep_ms(10); // Add a delay
        // lcd_send_byte(0x80, true); // Reset cursor to the first line

        // // Display the message
        // lcd_print(ir_message);
        // printf("IR Sensor: %.2fV\n", ir_voltage);

        // sleep_ms(1000); // Wait before next update
        //___________________________________________________________________________________________________________________

        // **2. Measure and Display Ultrasonic Sensor Distance**
        // float distance = measure_distance(TRIG_PIN, ECHO_PIN);
        // char us_message[32];
        // if (distance < 0) {
        //     sprintf(us_message, "No Obj Detected");
        //     printf("Ultrasonic: Timeout! No object detected.\n");
        // } else {
        //     sprintf(us_message, "Distance: %.2f cm", distance);
        //     printf("Ultrasonic: %.2f cm\n", distance);
        // }
        // lcd_print(us_message);
        // sleep_ms(2000);
        // lcd_send_byte(LCD_CLEAR, true);
        // sleep_ms(2000);
        
        //___________________________________________________________________________________________________________________

        // **4. Play a Buzzer Tone**
        // printf("Playing Buzzer Tone...\n");
        // lcd_print("     Playing Buzzer Tone...");
        // play_tone(BUZZER_PIN, 1000, 500); // Play 1kHz tone for 500ms
        // sleep_ms(2000);
        //___________________________________________________________________________________________________________________
        // **5. Simple Buzzer ON/OFF**
        //  printf("Playing 1kHz tone for 500ms\n");
        // buzzer_play_tone(BUZZER_PIN, 1000, 500); // 1kHz tone for 500ms
        // sleep_ms(500);                           // Wait for 500ms

        // // Play a 500Hz tone for 1 second
        // printf("Playing 500Hz tone for 1 second\n");
        // buzzer_play_tone(BUZZER_PIN, 500, 1000); // 500Hz tone for 1 second
        // sleep_ms(500);                           // Wait for 500ms

        // // Play a 2kHz tone for 250ms
        // printf("Playing 2kHz tone for 250ms\n");
        // buzzer_play_tone(BUZZER_PIN, 2000, 250); // 2kHz tone for 250ms
        // sleep_ms(5000); 
        //___________________________________________________________________________________________________________________
        // // **3. Rotate Servo Motor**
        // printf("Rotating Servo...\n");
        // uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);
        // rotate_servo(slice_num);

        // Repeat the loop
        printf("Cycle Completed\n");
    }

    return 0;
}