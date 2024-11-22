#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <stdint.h> // Ensure this is included for standard types

// Function prototypes
void ir_sensor_init_adc(uint32_t gpio_pin);
uint16_t ir_sensor_read_adc();
float ir_sensor_get_voltage(uint16_t raw_value);

#endif // IR_SENSOR_H
