#include <stdio.h>
#include "ir_sensor.h"
#include "pico/stdlib.h"
#include "hardware/adc.h"

// Initialize the IR sensor for ADC (analog mode)
void ir_sensor_init_adc(uint32_t gpio_pin) {
    adc_init();                  // Initialize ADC
    adc_gpio_init(gpio_pin);     // Enable ADC functionality on the GPIO

    // Select the ADC input based on the GPIO pin
    if (gpio_pin == 26) adc_select_input(0);
    else if (gpio_pin == 27) adc_select_input(1);
    else if (gpio_pin == 28) adc_select_input(2);
    else printf("Invalid GPIO for ADC\n");
}


// Read the raw ADC value from the IR sensor
uint16_t ir_sensor_read_adc() {
    return adc_read(); // Returns raw ADC value (0-4095)
}

// Convert the raw ADC value to a voltage
float ir_sensor_get_voltage(uint16_t raw_value) {
    return (raw_value * 3.3f) / 4095.0f; // Scale raw value to voltage (0-3.3V)
}
