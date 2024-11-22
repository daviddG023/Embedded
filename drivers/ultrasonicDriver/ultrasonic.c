#include "ultrasonic.h"
#include "pico/stdlib.h"

// Initialize the HC-SR04 ultrasonic sensor
void hc_sr04_init(uint32_t trig_pin, uint32_t echo_pin) {
    // Initialize TRIG as output
    gpio_init(trig_pin);
    gpio_set_dir(trig_pin, GPIO_OUT);
    gpio_put(trig_pin, 0); // Set TRIG low initially

    // Initialize ECHO as input
    gpio_init(echo_pin);
    gpio_set_dir(echo_pin, GPIO_IN);
}

// Measure the distance in cm
float measure_distance(uint32_t trig_pin, uint32_t echo_pin) {
    // Trigger a 10µs pulse on TRIG
    gpio_put(trig_pin, 1);
    sleep_us(10);
    gpio_put(trig_pin, 0);

    // Measure the pulse width on ECHO
    uint64_t start_time = time_us_64();
    while (!gpio_get(echo_pin)) { // Wait for ECHO to go HIGH
        if ((time_us_64() - start_time) > 2000000) return -1; // Timeout if no response
    }
    uint64_t echo_start = time_us_64();

    while (gpio_get(echo_pin)) { // Wait for ECHO to go LOW
        if ((time_us_64() - echo_start) > 2000000) return -1; // Timeout if no response
    }
    uint64_t echo_end = time_us_64();

    // Calculate distance in cm
    uint64_t pulse_duration = echo_end - echo_start;
    float distance = (pulse_duration * 0.0343) / 2; // Speed of sound = 343 m/s

    return distance;
}
