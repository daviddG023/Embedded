#ifndef BUZZER_H
#define BUZZER_H

#include "pico/stdlib.h"

void buzzer_init(uint gpio_pin);
void buzzer_play_tone(uint gpio_pin, uint frequency, uint duration_ms);

#endif // BUZZER_H
