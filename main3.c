#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "pico/multicore.h"


// FreeRTOS headers

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Peripheral Drivers
#include "drivers/lcdDriver/lcd.h"
#include "drivers/rfidDriver/rfid.h"
#include "drivers/buzzerDriver/buzzer.h"
#include "drivers/servoDriver/servo.h"
#include "drivers/irDriver/ir_sensor.h"
#include "drivers/ultrasonicDriver/ultrasonic.h"
#include "hardware/gpio.h"
#include "pwm.h"
#include <string.h>

// Define GPIO pins
#define BUZZER_PIN 26          // Buzzer GPIO pin
#define SERVO_PIN 20           // Servo GPIO pin
#define IR_SENSOR_ADC_PIN 27   // IR sensor ADC pin
#define TRIG_PIN 29            // Ultrasonic sensor TRIG pin
#define ECHO_PIN 28            // Ultrasonic sensor ECHO pin

// RFID Pin Definitions
#define SDA_PIN 21 // D9
#define SCK_PIN 6  // D13
#define MOSI_PIN 7 // D11
#define MISO_PIN 4 // D12
#define RST_PIN 5  // D10

// Mutex handle for LCD access
SemaphoreHandle_t xLCDMutex;

// Core-specific tasks
void core1_main();


// FreeRTOS Hooks
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    printf("Stack overflow detected in task: %s\n", pcTaskName);
    while (1);  // Halt the system
}

void vApplicationMallocFailedHook(void) {
    printf("Memory allocation failed!\n");
    while (1);  // Halt the system
}

void rfid_task(void *pvParameters) {
    MFRC522Ptr_t *mfrc = (MFRC522Ptr_t *)pvParameters;
    uint8_t uid[10];
    uint8_t uid_length;
    
    while (1) {
        // Display prompt on LCD
        xSemaphoreTake(xLCDMutex, portMAX_DELAY);
        lcd_send_byte(LCD_CLEAR, true); // Clear LCD screen
        lcd_print("Scan RFID Card");
        xSemaphoreGive(xLCDMutex);
        vTaskDelay(pdMS_TO_TICKS(2000));

        xSemaphoreTake(xLCDMutex, portMAX_DELAY);
        lcd_send_byte(LCD_CLEAR, true); // Clear LCD screen
        xSemaphoreGive(xLCDMutex);
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Check if a card is present
        if (card_present(*mfrc)) {
            if (read_card_uid(*mfrc, uid, &uid_length)) {
                // Format the UID as a string for display
                char uid_str[32];
                strcpy(uid_str, "UID: ");
                for (uint8_t i = 0; i < uid_length; i++) {
                    sprintf(uid_str + strlen(uid_str), "%02X ", uid[i]); // Append each byte in hex
                }

                // Update LCD with UID
                xSemaphoreTake(xLCDMutex, portMAX_DELAY);
                lcd_print(uid_str); // Print the UID on the LCD
                xSemaphoreGive(xLCDMutex);
                vTaskDelay(pdMS_TO_TICKS(1000));

                // Check UID for access control
                if (uid_length == 4 && uid[0] == 0x07 && uid[1] == 0x20 && uid[2] == 0xD1 && uid[3] == 0x26) {
                    // Matched UID - Play accessible sound
                    printf("Access Granted. Playing success tone...\n");
                    buzzer_play_tone(BUZZER_PIN, 1000, 500); // 1kHz tone for 500ms

                    // Rotate the servo
                    rotate_servo(pwm_gpio_to_slice_num(SERVO_PIN));

                    // Update LCD with success message
                    lcd_print("Access Granted");
                } else {
                    // Unmatched UID - Play non-accessible sound
                    printf("Access Denied. Playing error tone...\n");
                    buzzer_play_tone(BUZZER_PIN, 500, 1000); // 500Hz tone for 1 second

                    // Update LCD with denial message
                    lcd_print("Access Denied");
                }

                vTaskDelay(pdMS_TO_TICKS(5000));
            }
        } else {
            // No card present
            xSemaphoreTake(xLCDMutex, portMAX_DELAY);
            lcd_print("No Card Present");
            xSemaphoreGive(xLCDMutex);
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
}

// Core 0 Task: IR Sensor
void ir_sensor_task(void *pvParameters) {
    while (1) {
        uint16_t ir_raw_value = ir_sensor_read_adc();
        float ir_voltage = ir_sensor_get_voltage(ir_raw_value);

        char ir_message[32];
        snprintf(ir_message, sizeof(ir_message), "IR: %.2fV", ir_voltage);

        xSemaphoreTake(xLCDMutex, portMAX_DELAY);
        lcd_send_byte(LCD_CLEAR, true);
        lcd_print(ir_message);
        xSemaphoreGive(xLCDMutex);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Core 0 Task: Ultrasonic Sensor
void ultrasonic_task(void *pvParameters) {
    while (1) {
        float distance = measure_distance(TRIG_PIN, ECHO_PIN);
        char us_message[32];
        snprintf(us_message, sizeof(us_message), "Dist: %.2fcm", distance);

        xSemaphoreTake(xLCDMutex, portMAX_DELAY);
        lcd_print(us_message);
        xSemaphoreGive(xLCDMutex);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// Initialization Task
void initialization_task(void *pvParameters) {
    stdio_init_all();
    lcd_init();

    buzzer_init(BUZZER_PIN);
    servo_init(SERVO_PIN);
    ir_sensor_init_adc(IR_SENSOR_ADC_PIN);
    hc_sr04_init(TRIG_PIN, ECHO_PIN);

    xLCDMutex = xSemaphoreCreateMutex();
    if (xLCDMutex == NULL) {
        printf("Failed to create LCD mutex.\n");
        vTaskDelete(NULL);
    }

    lcd_print("System Ready!");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Tasks for Core 0
    xTaskCreate(ir_sensor_task, "IR Sensor Task", 512, NULL, 1, NULL);
    xTaskCreate(ultrasonic_task, "Ultrasonic Task", 512, NULL, 1, NULL);

    vTaskDelete(NULL); // End initialization task
}

// Core 1 Main Function
void core1_main() {
    // RFID Task on Core 1
    xTaskCreate(rfid_task, "RFID Task", 1024, NULL, 2, NULL);

    // Start FreeRTOS scheduler on Core 1
    vTaskStartScheduler();
}

// Main Function
int main() {
    // Launch Core 1
    multicore_launch_core1(core1_main);

    // Initialization Task for Core 0
    xTaskCreate(initialization_task, "Initialization Task", 2048, NULL, 3, NULL);

    // Start FreeRTOS scheduler on Core 0
    vTaskStartScheduler();

    while (1) { }
    return 0;
}