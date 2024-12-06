cmake_minimum_required(VERSION 3.13...3.27)

# Initialize the Raspberry Pi Pico SDK
include(pico_sdk_import.cmake)
project(MS3)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Define the executable and include the main source file
add_executable(MS3  # Change the executable name to MS3
    main.c
    drivers/buzzerDriver/buzzer.c
    drivers/servoDriver/servo.c
    drivers/irDriver/ir_sensor.c
    drivers/lcdDriver/lcd.c
    drivers/ultrasonicDriver/ultrasonic.c
    drivers/rfidDriver/rfid.c
    drivers/rfidDriver/mfrc522.c
    freertos/tasks.c
    freertos/queue.c
    freertos/portable/GCC/ARM_CM0/port.c
    freertos/portable/MemMang/heap_4.c
)

# Include directories for drivers and any additional headers
target_include_directories(MS3 PRIVATE
    freertos/include
    freertos/portable/GCC/ARM_CM0
    drivers/
    drivers/buzzerDriver
    drivers/servoDriver
    drivers/irDriver
    drivers/lcdDriver
    drivers/ultrasonicDriver
    drivers/rfidDriver
    include/hardware
    ${CMAKE_CURRENT_SOURCE_DIR} # Include the directory containing pwm.h (same as main.c)
)

# Link necessary libraries
target_link_libraries(MS3
    pico_stdlib
    hardware_pwm
    hardware_adc
    hardware_gpio
    hardware_timer
    hardware_spi
)

# Enable additional output formats (UF2, bin, etc.)
pico_add_extra_outputs(MS3)

# Enable USB if needed (optional, for debugging or communication purposes)
pico_enable_stdio_usb(MS3 1)
pico_enable_stdio_uart(MS3 0)
