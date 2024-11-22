#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

// Function prototypes
void servo_init(uint gpio_pin);
void set_servo_angle(uint slice_num, uint angle);
void rotate_servo(uint slice_num);

#endif // SERVO_H
