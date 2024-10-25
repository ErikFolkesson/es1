#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"

#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "inc/tm4c129encpdt.h"

// How long it takes to simulate work being done by tasks.
#define WAIT_ITERATIONS 10000000

SemaphoreHandle_t g_printMutex;

void ConfigureUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    uint32_t baudRate = 115200;
    // 8 data bits, 1 stop bit, and no parity bit.
    uint32_t uartConfig = UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE
            | UART_CONFIG_PAR_NONE;
    // For PIOSC clock, the frequency should be set as 16'000'000
    uint32_t systemClockFrequency = 16000000;
    UARTConfigSetExpClk(UART0_BASE, systemClockFrequency, baudRate, uartConfig);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    UARTEnable(UART0_BASE);
}

void uartPuts(const char *str)
{
    const char *it;
    for (it = str; *it != '\0'; it++)
    {
        UARTCharPut(UART0_BASE, *it);
    }
}

typedef struct
{
    SemaphoreHandle_t mutex; // The mutex
    uint16_t releaseTimeMs; // How many milliseconds to wait before the blinking should start.
    uint16_t periodMs; // How long each period should be.
    const char *name;
} TaskArgs;

// The function executed by the low and high priority tasks.
// It takes a mutex, does some work, and then releases the mutex.
void vTaskTakeMutex(void *pvParameters)
{
    const TaskArgs *args = pvParameters;

    vTaskDelay(pdMS_TO_TICKS(args->releaseTimeMs));

    TickType_t startTimePoint = xTaskGetTickCount();

    while (true)
    {
        // The writes to UART are protected with a mutex.
        xSemaphoreTake(g_printMutex, portMAX_DELAY);
        uartPuts("Task ");
        uartPuts(args->name);
        uartPuts(" started\r\n");
        xSemaphoreGive(g_printMutex);

        // Take the mutex, from here until we release the mutex priority inversion is a risk.
        xSemaphoreTake(args->mutex, portMAX_DELAY);

        xSemaphoreTake(g_printMutex, portMAX_DELAY);
        uartPuts("Task ");
        uartPuts(args->name);
        uartPuts(" sem take\r\n");
        xSemaphoreGive(g_printMutex);

        xSemaphoreTake(g_printMutex, portMAX_DELAY);
        uartPuts("Task ");
        uartPuts(args->name);
        uartPuts(" started its workload\r\n");
        xSemaphoreGive(g_printMutex);

        // Simulate some work being done.
        volatile int i = 0;
        while (i < WAIT_ITERATIONS)
        {
            i++;
        }

        xSemaphoreTake(g_printMutex, portMAX_DELAY);
        uartPuts("Task ");
        uartPuts(args->name);
        uartPuts(" sem give\r\n");
        xSemaphoreGive(g_printMutex);

        xSemaphoreGive(args->mutex);

        xSemaphoreTake(g_printMutex, portMAX_DELAY);
        uartPuts("Task ");
        uartPuts(args->name);
        uartPuts(" finished\r\n");
        xSemaphoreGive(g_printMutex);

        // Check if the task has missed its deadline for the period.
        TickType_t currentTimePoint = xTaskGetTickCount();
        TickType_t expectedTimePoint = startTimePoint
                + pdMS_TO_TICKS(args->periodMs);
        int32_t difference = (int32_t) currentTimePoint
                - (int32_t) expectedTimePoint;
        if (difference > 0)
        {
            xSemaphoreTake(g_printMutex, portMAX_DELAY);
            uartPuts("Task ");
            uartPuts(args->name);
            uartPuts(" missed its deadline by ");

            char buf[30] = { 0 };
            sprintf(buf, "%d", difference);
            uartPuts(buf);
            uartPuts("ms\r\n");

            xSemaphoreGive(g_printMutex);
        }

        vTaskDelayUntil(&startTimePoint, args->periodMs);
    }
}

// The function for the medium priority task, which just performs some work without taking any mutex.
void vTaskBusyWork(void *pvParameters)
{
    const TaskArgs *args = pvParameters;

    vTaskDelay(pdMS_TO_TICKS(args->releaseTimeMs));

    TickType_t startTimePoint = xTaskGetTickCount();

    while (true)
    {
        xSemaphoreTake(g_printMutex, portMAX_DELAY);
        uartPuts("Task ");
        uartPuts(args->name);
        uartPuts(" started\r\n");
        xSemaphoreGive(g_printMutex);

        xSemaphoreTake(g_printMutex, portMAX_DELAY);
        uartPuts("Task ");
        uartPuts(args->name);
        uartPuts(" started its workload\r\n");
        xSemaphoreGive(g_printMutex);

        // Simulate some work being done.
        volatile int i = 0;
        while (i < WAIT_ITERATIONS)
        {
            i++;
        }

        xSemaphoreTake(g_printMutex, portMAX_DELAY);
        uartPuts("Task ");
        uartPuts(args->name);
        uartPuts(" finished\r\n");
        xSemaphoreGive(g_printMutex);

        // Check if the task has missed its deadline for the period.
        TickType_t currentTimePoint = xTaskGetTickCount();
        TickType_t expectedTimePoint = startTimePoint
                + pdMS_TO_TICKS(args->periodMs);
        int32_t difference = (int32_t) currentTimePoint
                - (int32_t) expectedTimePoint;
        if (difference > 0)
        {
            xSemaphoreTake(g_printMutex, portMAX_DELAY);
            uartPuts("Task ");
            uartPuts(args->name);
            uartPuts(" missed its deadline by ");

            char buf[30] = { 0 };
            sprintf(buf, "%d", difference);
            uartPuts(buf);
            uartPuts("ms\r\n");

            xSemaphoreGive(g_printMutex);
        }

        vTaskDelayUntil(&startTimePoint, pdMS_TO_TICKS(args->periodMs));
    }
}

// Prints the ANSI escape sequence to move the cursor to the home position (1,1)
void clearScreenAndMoveCursorHome(void)
{
    uartPuts("\x1B[2J");
    uartPuts("\x1B[H");
}

int main(void)
{
    ConfigureUART();

    // Set the system clock to the same as in FreeRTOSConfig.h.
    SysCtlClockFreqSet(
    SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480,
                       SYSTEM_CLOCK);

    clearScreenAndMoveCursorHome();

    uartPuts("\r\nStarting new run\r\n");

    // Create mutexes and binaries
    SemaphoreHandle_t mutex = xSemaphoreCreateCounting(1, 1);
    assert(mutex);

    g_printMutex = xSemaphoreCreateMutex();

    const uint16_t taskStackDepth = 1000;
    const UBaseType_t lowPrio = tskIDLE_PRIORITY + 1;
    const UBaseType_t mediumPrio = tskIDLE_PRIORITY + 2;
    const UBaseType_t highPrio = tskIDLE_PRIORITY + 3;

    // Construct task arguments as static variables to make sure they outlive the tasks.
    // The release times are in the order of low -> high -> mid.
    static TaskArgs lowPriorityArgs;
    lowPriorityArgs = (TaskArgs ) { .mutex = mutex, .releaseTimeMs = 0,
                                    .periodMs = 4000, .name = "low" };

    static TaskArgs mediumPriorityArgs;
    mediumPriorityArgs = (TaskArgs ) { .mutex = mutex, .releaseTimeMs = 400,
                                       .periodMs = 4000, .name = "mid" };

    static TaskArgs highPriorityArgs;
    highPriorityArgs = (TaskArgs ) { .mutex = mutex, .releaseTimeMs = 200,
                                     .periodMs = 4000, .name = "high" };

    BaseType_t result;
    result = xTaskCreate(vTaskTakeMutex, "LowPrioTask", taskStackDepth,
                         &lowPriorityArgs, lowPrio, NULL);
    assert(result == pdPASS);

    result = xTaskCreate(vTaskBusyWork, "MidPrioTask", taskStackDepth,
                         &mediumPriorityArgs, mediumPrio, NULL);
    assert(result == pdPASS);

    result = xTaskCreate(vTaskTakeMutex, "HighPrioTask", taskStackDepth,
                         &highPriorityArgs, highPrio, NULL);
    assert(result == pdPASS);

    // Start the scheduler with vTaskStartScheduler
    vTaskStartScheduler();

    // We should never reach here since vTaskStartScheduler should block.
    assert(false);
    while (true)
    {
    }
}

