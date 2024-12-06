#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "servo.h"

static uint slice_num_global; // Store slice number for the servo

// Initialize the servo motor
void servo_init(uint gpio_pin) {
    // Set up GPIO for PWM
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM); // Set GPIO pin as PWM
    slice_num_global = pwm_gpio_to_slice_num(gpio_pin); // Get PWM slice

    // Set PWM frequency to 50Hz (20ms period)
    pwm_set_wrap(slice_num_global, 20000); // 20,000 ticks for 50Hz frequency
    pwm_set_clkdiv(slice_num_global, 64.0f); // Set clock divider
    pwm_set_enabled(slice_num_global, true); // Enable PWM on the slice

    printf("Servo Motor Initialized on GPIO %d\n", gpio_pin);
}

// Set the servo angle (0° to 180°)
void set_servo_angle(uint slice_num, uint angle) {
    // Map angle (0° to 180°) to PWM duty cycle (500-2500 microseconds)
    uint duty = 500 + (angle * 2000 / 180);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, duty);
    printf("Setting angle: %d°, Duty cycle: %d\n", angle, duty);
}


void rotate_servo(uint slice_num) {
    uint64_t start_time = to_ms_since_boot(get_absolute_time()); // Record the start time

    while (to_ms_since_boot(get_absolute_time()) - start_time < 5000) { // Run for 5 seconds
        // Gradually increase angle from 0° to 180°
        for (int angle = 0; angle <= 180; angle += 5) {
            set_servo_angle(slice_num, angle);
            sleep_ms(50); // Wait for the servo to reach the position

            // Check if 5 seconds have elapsed
            if (to_ms_since_boot(get_absolute_time()) - start_time >= 5000) {
                return;
            }
        }

        // Gradually decrease angle from 180° to 0°
        for (int angle = 180; angle >= 0; angle -= 5) {
            set_servo_angle(slice_num, angle);
            sleep_ms(50); // Wait for the servo to reach the position

            // Check if 5 seconds have elapsed
            if (to_ms_since_boot(get_absolute_time()) - start_time >= 5000) {
                return;
            }
        }
    }
}

// Function to open the servo from 90° to 0°
void open_servo(uint slice_num) {
    // Gradually decrease angle from 90° to 0°
    for (int angle = 90; angle >= 0; angle -= 5) {
        set_servo_angle(slice_num, angle);
        sleep_ms(50); // Wait for the servo to reach the position
    }
    printf("Servo opened from 90° to 0°.\n");
}

// Function to close the servo from 0° to 90°
void close_servo(uint slice_num) {
    // Gradually increase angle from 0° to 90°
    for (int angle = 0; angle <= 90; angle += 5) {
        set_servo_angle(slice_num, angle);
        sleep_ms(50); // Wait for the servo to reach the position
    }
    printf("Servo closed from 0° to 90°.\n");
}