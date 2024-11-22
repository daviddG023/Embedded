#include "buzzer.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

void buzzer_init(uint gpio_pin) {
    gpio_set_function(gpio_pin, GPIO_FUNC_PWM);  // Set GPIO to PWM
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_set_enabled(slice_num, true);  // Enable PWM on the slice
}

void buzzer_play_tone(uint gpio_pin, uint frequency, uint duration_ms) {
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    uint clock_divider = 4; // Adjust as needed for precision
    uint32_t top = 125000000 / (frequency * clock_divider); // Calculate the top value
    
    pwm_set_clkdiv(slice_num, clock_divider); // Set the clock divider
    pwm_set_wrap(slice_num, top);            // Set the top value
    pwm_set_gpio_level(gpio_pin, top / 2);   // 50% duty cycle for tone

    sleep_ms(duration_ms);                   // Play for specified duration
    pwm_set_gpio_level(gpio_pin, 0);         // Turn off the buzzer
}
