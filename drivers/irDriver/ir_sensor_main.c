#include <stdio.h>
#include "ir_sensor.h"

#define IR_SENSOR_ADC_PIN 26 // ADC pin for IR sensor

// int main() {
//     stdio_init_all(); // Initialize standard I/O for debugging
//     printf("Analog IR Sensor Test\n");

//     // Initialize the IR sensor in ADC mode
//     ir_sensor_init_adc(IR_SENSOR_ADC_PIN);

//     while (true) {
//         // Read raw ADC value
//         uint16_t raw_value = ir_sensor_read_adc();

//         // Convert raw value to voltage
//         float voltage = ir_sensor_get_voltage(raw_value);

//         // Print the raw value and voltage
//         printf("IR Sensor Value: %u, Voltage: %.2fV\n", raw_value, voltage);

//         sleep_ms(500); // Delay for readability
//     }

//     return 0;
// }
