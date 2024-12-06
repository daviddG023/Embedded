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

#include "./freertos/include/FreeRTOS.h"
#include "./freertos/include/task.h"
#include "./freertos/include/queue.h"
#include "./freertos/include/semphr.h"


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


TaskHandle_t rfidTaskHandle, irSensorTaskHandle, ultrasonicTaskHandle;
QueueHandle_t lcdQueue;

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

void rfid_task(void *pvParameters) {
    MFRC522Ptr_t mfrc = (MFRC522Ptr_t)pvParameters;

    uint8_t uid[10];
    uint8_t uid_length;
    char lcdMessage[32];

    while (1) {
        // Check if a card is present
        if (card_present(mfrc)) {
            if (read_card_uid(mfrc, uid, &uid_length)) {
                char uid_str[30];
                sprintf(uid_str, "UID: ");
                for (uint8_t i = 0; i < uid_length; i++) {
                    sprintf(uid_str + strlen(uid_str), "%02X ", uid[i]); // Append each byte in hex
                }

                if (uid_length == 4 && uid[0] == 0x07 && uid[1] == 0x20 && uid[2] == 0xD1 && uid[3] == 0x26) {
                    // Matched UID - Play accessible sound
                    buzzer_play_tone(BUZZER_PIN, 1000, 1500);
                    open_servo(pwm_gpio_to_slice_num(SERVO_PIN));
                    // rotate_servo(pwm_gpio_to_slice_num(SERVO_PIN));
                    snprintf(lcdMessage, sizeof(lcdMessage), "Access Granted");
                } else {
                    // Unmatched UID - Play non-accessible sound
                    buzzer_play_tone(BUZZER_PIN, 500, 1000);
                    snprintf(lcdMessage, sizeof(lcdMessage), "Access Denied");
                }

                // Send message to LCD queue
                xQueueSend(lcdQueue, lcdMessage, portMAX_DELAY);
            }
        } else {
            snprintf(lcdMessage, sizeof(lcdMessage), "No Card Present");
            xQueueSend(lcdQueue, lcdMessage, portMAX_DELAY);
        }

        // Delay to prevent spamming the RFID reader
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void ir_sensor_task(void *pvParameters) {
    char lcdMessage[32];
    while (1) {
        uint16_t ir_raw_value = ir_sensor_read_adc();
        float ir_voltage = ir_sensor_get_voltage(ir_raw_value);

        if (ir_voltage > 3.3 || ir_voltage < 0) {
            snprintf(lcdMessage, sizeof(lcdMessage), "Invalid IR");
        } else {
            snprintf(lcdMessage, sizeof(lcdMessage), "IR: %.2fV", ir_voltage);
        }

        // Send message to LCD queue
        xQueueSend(lcdQueue, lcdMessage, portMAX_DELAY);

        // Print to serial for debugging
        printf("IR Sensor: %.2fV\n", ir_voltage);

        // Delay for 2 seconds
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
void lcd_task(void *pvParameters) {
    char lcdMessage[32];

    while (1) {
        // Wait for a message from any task
        if (xQueueReceive(lcdQueue, lcdMessage, portMAX_DELAY)) {
            lcd_send_byte(LCD_CLEAR, true); // Clear the LCD
            lcd_print(lcdMessage);         // Display the message
        }
    }
}

void ultrasonic_task(void *pvParameters) {
    char lcdMessage[32];
    while (1) {
        float distance = measure_distance(TRIG_PIN, ECHO_PIN);

        if (distance < 0) {
            snprintf(lcdMessage, sizeof(lcdMessage), "No Obj Detected");
        } else {
            if (distance < 10) {
                snprintf(lcdMessage, sizeof(lcdMessage), "Dist: %.2fcm", distance);
                // Close the gate and wait to avoid immediate reopening
                printf("Object detected within 10cm. Closing gate...\n");
                close_servo(pwm_gpio_to_slice_num(SERVO_PIN)); // Close gate
                vTaskDelay(pdMS_TO_TICKS(3000)); // 3-second delay before the next check
            } else {
                snprintf(lcdMessage, sizeof(lcdMessage), "Dist: %.2fcm", distance);
            }
        }

        // Send message to LCD queue
        xQueueSend(lcdQueue, lcdMessage, portMAX_DELAY);

        // Print to serial for debugging
        printf("Ultrasonic: %.2f cm\n", distance);

        // Delay for 2 seconds
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    printf("Stack overflow detected in task: %s\n", pcTaskName);
    while (1);  // Halt the system
}
void vApplicationMallocFailedHook(void) {
    printf("Memory allocation failed!\n");
    while (1);  // Halt the system
}


int main() {
    // Initialize peripherals
    MFRC522Ptr_t mfrc;
    setup_all(&mfrc);

    // Create LCD queue
    lcdQueue = xQueueCreate(5, sizeof(char) * 32); // Queue for LCD messages

    // Create tasks
    xTaskCreate(rfid_task, "RFID Task", 1024, (void *)mfrc, 2, &rfidTaskHandle);
    xTaskCreate(ir_sensor_task, "IR Sensor Task", 1024, NULL, 1, &irSensorTaskHandle);
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 1024, NULL, 1, &ultrasonicTaskHandle);
    xTaskCreate(lcd_task, "LCD Task", 1024, NULL, 3, NULL);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // Should never reach here
    while (1) {}

    return 0;
}
