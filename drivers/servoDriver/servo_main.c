#include <stdio.h>

#include "servo.h"


#define SERVO_PIN 28 // GPIO where the servo signal is connected

/*int main() {
    stdio_init_all(); // Initialize standard input/output for debugging
    printf("Servo Motor Test Program\n");

    // Initialize the servo motor
    servo_init(SERVO_PIN);

    // Get the PWM slice number for the specified GPIO
    uint slice_num = pwm_gpio_to_slice_num(SERVO_PIN);

    // Rotate the servo
    rotate_servo(slice_num);

    return 0; // This will never execute due to the infinite loop in `rotate_servo`
}*/
