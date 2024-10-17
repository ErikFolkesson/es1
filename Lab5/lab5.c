#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "drivers/buttons.h"
#include "drivers/pinout.h"
#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"

typedef struct
{
    SemaphoreHandle_t *ledMutex; // The mutex for modifying/holding the LED.
    uint16_t blinkDuration; // How many microseconds the LED should spend in each on/off state.
    uint8_t ledId; // CLP_D1 .. CLP_D4.
} TaskBlinkArgs;

typedef struct
{
    SemaphoreHandle_t *ledMutex; // The mutex for modifying/holding the LED.
    SemaphoreHandle_t *buttonSemaphore; // The semaphore which signals a button press.
    uint8_t ledId; // CLP_D1 .. CLP_D4.
} TaskHoldArgs;

void vTaskBlink(void *pvParameters)
{
    const TaskBlinkArgs *args = pvParameters;
    // Init specified LED

    // Synch, maybe

    while (true)
    {
        // Task code

    }
}

void vTaskHold(void *pvParameters)
{
    const TaskHoldArgs *args = pvParameters;
// Take unavailable semaphore
// Lower priority
    while (true)
    {
        // Task code
    }
}

// FixMe: Interrupt handler for the buttons?

int main(void)
{
    // Create mutexes and binaries
    SemaphoreHandle_t LED_mutex_1 = xSemaphoreCreateMutex();
    SemaphoreHandle_t LED_mutex_2 = xSemaphoreCreateMutex();

    SemaphoreHandle_t button_sem_1 = xSemaphoreCreateBinary(); // FixMe: Maybe should be: xSemaphoreCreateCounting(1, 0);
    SemaphoreHandle_t button_sem_2 = xSemaphoreCreateBinary();

    // Init LEDs
    // Configure the device pins.
    PinoutSet(false, false);

    // Initialize the button driver.
    ButtonsInit();

    // Enable the GPIO pin for the LED (PN0 - PN3).
    // Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE,
                          GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    // Construct task arguments as static variables to make sure they outlive the tasks.
    static TaskBlinkArgs blink1Args;
    blink1Args = (TaskBlinkArgs ) { .ledMutex = &LED_mutex_1, .blinkDuration =
                                            1000,
                                    .ledId = CLP_D1 };
    static TaskBlinkArgs blink2Args;
    blink2Args = (TaskBlinkArgs ) { .ledMutex = &LED_mutex_2, .blinkDuration =
                                            2000,
                                    .ledId = CLP_D2 };
    static TaskBlinkArgs blink3Args;
    blink3Args = (TaskBlinkArgs ) { .ledMutex = NULL, .blinkDuration = 3000,
                                    .ledId = CLP_D3 };
    static TaskBlinkArgs blink4Args;
    blink4Args = (TaskBlinkArgs ) { .ledMutex = NULL, .blinkDuration = 4000,
                                    .ledId = CLP_D4 };

    static TaskHoldArgs hold1Args;
    hold1Args = (TaskHoldArgs ) { .ledMutex = &LED_mutex_1, .buttonSemaphore =
                                          &button_sem_1,
                                  .ledId = CLP_D1 };
    static TaskHoldArgs hold2Args;
    hold2Args = (TaskHoldArgs ) { .ledMutex = &LED_mutex_2, .buttonSemaphore =
                                          &button_sem_2,
                                  .ledId = CLP_D2 };

    // Create tasks
    // CLP_D1, CLP_D2, CLP_D3, CLP_D4
    // FixME: Should use xTaskCreate instead.
    blink_LED_1 = vTaskBlink(CLP_D1, 1000, LED_mutex_1);
    blink_LED_2 = vTaskBlink(CLP_D2, 2000, LED_mutex_2);
    blink_LED_3 = vTaskBlink(CLP_D3, 3000, null);
    blink_LED_4 = vTaskBlink(CLP_D4, 4000, null);

    // FixME: Placeholder
    hold_LED_1 = vTaskHold(CLP_D1);
    hold_LED_2 = vTaskHold(CLP_D2);

    // Start the scheduler with vTaskStartScheduler


    while(true) {
        // poll buttons
        // We have to have some delay here else we risk starvation.
    }

    return 0;
}

