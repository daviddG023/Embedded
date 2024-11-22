#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h> // Include this for standard integer types

// Function prototypes
void hc_sr04_init(uint32_t trig_pin, uint32_t echo_pin);
float measure_distance(uint32_t trig_pin, uint32_t echo_pin);

#endif // ULTRASONIC_H
