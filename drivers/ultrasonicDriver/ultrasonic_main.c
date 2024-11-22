#include <stdio.h>
#include "ultrasonic.h"

#define TRIG_PIN 3  // Pin connected to HC-SR04 TRIG
#define ECHO_PIN 2  // Pin connected to HC-SR04 ECHO

/*int main() {
    stdio_init_all();
    hc_sr04_init(TRIG_PIN, ECHO_PIN);
    printf("HC-SR04 Ultrasonic Sensor Initialized\n");

    while (true) {
        float distance = measure_distance(TRIG_PIN, ECHO_PIN);

        if (distance < 0) {
            printf("Timeout! No object detected.\n");
        } else {
            printf("Distance: %.2f cm\n", distance);
        }

        sleep_ms(1000); // Wait 1 second before the next reading
    }

    return 0;
}*/
