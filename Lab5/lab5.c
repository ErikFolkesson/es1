#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "drivers/buttons.h"
#include "drivers/pinout.h"

void vTaskBlink(void *pvParameters)
{
    // Init specified LED

    // Synch, maybe

    while (true)
    {
        // Task code

    }
}

void vTaskHold(void *pvParameters)
{
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

