#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

// Basic configurations
#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      125000000  // 125 MHz for RP2040
#define configTICK_RATE_HZ                      1000       // 1 ms tick rate
#define configMAX_PRIORITIES                    5
#define configMINIMAL_STACK_SIZE                128
#define configTOTAL_HEAP_SIZE                   (32 * 1024)  // 32 KB heap
#define configMAX_TASK_NAME_LEN                 16
#define configENABLE_MPU                        0

#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1

// Hook functions
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_MALLOC_FAILED_HOOK            1

// Enable dynamic and static task creation
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configSUPPORT_STATIC_ALLOCATION         0

// Debugging options
#define configCHECK_FOR_STACK_OVERFLOW          2

// Cortex-M specific definitions
#define configPRIO_BITS                         2  // Cortex-M0 priority bits
#define configKERNEL_INTERRUPT_PRIORITY         (3 << (8 - configPRIO_BITS))  // Lowest priority
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    (1 << (8 - configPRIO_BITS))  // High priority for syscalls

// Enable these to map FreeRTOS handlers to CMSIS handlers
#define vPortSVCHandler                         SVC_Handler
#define xPortPendSVHandler                      PendSV_Handler
#define xPortSysTickHandler                     SysTick_Handler

// Task notifications (optional but useful for modern FreeRTOS)
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   3

// Software timers
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               2
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            256

// API configurations
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_vTaskDelayUntil                 1

#endif /* FREERTOS_CONFIG_H */
