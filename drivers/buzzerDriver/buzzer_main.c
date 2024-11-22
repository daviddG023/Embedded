#include <stdio.h>

#include "buzzer.h"

#define BUZZER_PIN 15 // Pin connected to the buzzer

/*int main() {
    stdio_init_all();
    buzzer_init(BUZZER_PIN); // Initialize the buzzer

    printf("Buzzer Test Initialized\n");

    while (true) {
        // Play a 1kHz tone for 500ms
        printf("Playing tone: 1kHz for 500ms\n");
        play_tone(BUZZER_PIN, 1000, 500);

        // Wait for 500ms
        sleep_ms(500);

        // Turn the buzzer ON for 500ms (simple mode)
        printf("Buzzer ON (Simple Mode)\n");
        buzzer_on(BUZZER_PIN);
        sleep_ms(500);

        // Turn the buzzer OFF
        printf("Buzzer OFF (Simple Mode)\n");
        buzzer_off(BUZZER_PIN);
        sleep_ms(500);
    }

    return 0;
}*/
