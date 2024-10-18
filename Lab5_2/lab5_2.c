#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "drivers/buttons.h"
#include "drivers/pinout.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"

#include "driverlib/uart.h"
#include "utils/uartstdio.h"

// Configure the UART.
void ConfigureUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 115200, 16000000);
}

typedef struct
{
    SemaphoreHandle_t mutex; // The mutex
    uint16_t startOffsetSeconds; // How many seconds to wait before the blinking should start.
} TaskArgs;

void vTaskTakeMutex(void *pvParameters)
{
    const TaskArgs *args = pvParameters;

    TickType_t startTimePoint = xTaskGetTickCount();

    vTaskDelayUntil(&startTimePoint, args->startOffsetSeconds);

    while (true)
    {
        // The task will be blocked here until the handler for the button press releases the semaphore.
        xSemaphoreTake(args->mutex, portMAX_DELAY);

        volatile int i = 0;

        while (i < 600000)
        {
            i++;
        }

        // Prevent the handler for the button press from giving the semaphore while we're holding the LED.
        xSemaphoreGive(args->mutex);
    }
}

void vTaskBusyWork(void *pvParameters)
{
    const TaskArgs *args = pvParameters;

    TickType_t startTimePoint = xTaskGetTickCount();

    vTaskDelayUntil(&startTimePoint, args->startOffsetSeconds);

    while (true)
    {
        volatile int i = 0;

        while (i < 600000)
        {
            i++;
        }
    }
}

int main(void)
{
    ConfigureUART();

    // Set the system clock to the same as in FreeRTOSConfig.h.
    SysCtlClockFreqSet(
    SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480,
                       SYSTEM_CLOCK);

    // Create mutexes and binaries
    SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

    const uint16_t taskStackDepth = 1000;
    const UBaseType_t lowPrio = tskIDLE_PRIORITY + 1;
    const UBaseType_t mediumPrio = tskIDLE_PRIORITY + 2;
    const UBaseType_t highPrio = tskIDLE_PRIORITY + 3;

    // Construct task arguments as static variables to make sure they outlive the tasks.
    static TaskArgs lowPriorityArgs;
    lowPriorityArgs = (TaskArgs ) { .mutex = mutex, .startOffsetSeconds = 0 };

    static TaskArgs mediumPriorityArgs;
    mediumPriorityArgs = (TaskArgs ) { .mutex = mutex, .startOffsetSeconds =
                                               2000 };

    static TaskArgs highPriorityArgs;
    highPriorityArgs =
            (TaskArgs ) { .mutex = mutex, .startOffsetSeconds = 1000 };

    BaseType_t result;
    result = xTaskCreate(vTaskTakeMutex, "LowPrioTask", taskStackDepth,
                         &lowPriorityArgs, lowPrio, NULL);
    assert(result == pdPASS);

    result = xTaskCreate(vTaskBusyWork, "MediumPrioTask", taskStackDepth,
                         &mediumPriorityArgs, mediumPrio, NULL);
    assert(result == pdPASS);

    result = xTaskCreate(vTaskTakeMutex, "HighPrioTask", taskStackDepth,
                         &highPriorityArgs, highPrio, NULL);
    assert(result == pdPASS);

    UARTprintf("\033[2JEnter text: ");

    // Start the scheduler with vTaskStartScheduler
    vTaskStartScheduler();

    // We should never reach here since vTaskStartScheduler should block.
    assert(false);
    while (true)
    {
    }
}

