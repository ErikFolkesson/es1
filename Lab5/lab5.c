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

typedef struct
{
    SemaphoreHandle_t ledMutex; // The mutex for modifying/holding the LED.
    uint16_t blinkDuration; // How many milliseconds the LED should spend in each on/off state.
    uint8_t startOffsetSeconds; // How many seconds to wait before the blinking should start.
    uint8_t ledId; // CLP_D1 .. CLP_D4.
} TaskBlinkArgs;

typedef struct
{
    SemaphoreHandle_t ledMutex; // The mutex for modifying/holding the LED.
    SemaphoreHandle_t buttonSemaphore; // The semaphore which signals a button press.
    uint8_t ledId; // CLP_D1 .. CLP_D4.
} TaskHoldArgs;

typedef struct
{
    SemaphoreHandle_t leftButtonSemaphore; // The semaphore which signals the left button press.
    SemaphoreHandle_t rightButtonSemaphore; // The semaphore which signals the right button press.
} ButtonHandlerArgs;

void vTaskBlink(void *pvParameters)
{
    const TaskBlinkArgs *args = pvParameters;

    // Get start time
    TickType_t currentWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&currentWakeTime,
                    pdMS_TO_TICKS(args->startOffsetSeconds * 1000));

    if (args->ledMutex == NULL)
    {
        while (true)
        {
            // Turn on LED args.ledId
            LEDWrite((args->ledId), (args->ledId));

            // Wait for args.blinkDuration ms
            vTaskDelayUntil(&currentWakeTime,
                            pdMS_TO_TICKS(args->blinkDuration));

            // Turn off LED args.ledId
            LEDWrite((args->ledId), 0);

            // Wait for args.blinkDuration ms
            vTaskDelayUntil(&currentWakeTime,
                            pdMS_TO_TICKS(args->blinkDuration));
        }
    }
    else
    {
        while (true)
        {
            // take mutex without blocking
            BaseType_t success = xSemaphoreTake(args->ledMutex, 0);

            if (success)
            {
                // Turn on LED args.ledId
                LEDWrite((args->ledId), (args->ledId));
                xSemaphoreGive(args->ledMutex);
            }

            // Wait for args.blinkDuration m
            vTaskDelayUntil(&currentWakeTime,
                            pdMS_TO_TICKS(args->blinkDuration));

            success = xSemaphoreTake(args->ledMutex, 0);
            if (success)
            {
                // Turn off LED args.ledId
                LEDWrite((args->ledId), 0);
                xSemaphoreGive(args->ledMutex);
            }

            // Wait for args.blinkDuration ms
            vTaskDelayUntil(&currentWakeTime,
                            pdMS_TO_TICKS(args->blinkDuration));
        }
    }
}

void vTaskHold(void *pvParameters)
{
    const TaskHoldArgs *args = pvParameters;
    const TickType_t sleepDuration = pdMS_TO_TICKS(10000);

    while (true)
    {
        // The task will be blocked here until the handler for the button press releases the semaphore.
        xSemaphoreTake(args->buttonSemaphore, portMAX_DELAY);
        // Prevent the handler for the button press from giving the semaphore while we're holding the LED.
        xSemaphoreGive(args->buttonSemaphore);

        TickType_t startTimePoint = xTaskGetTickCount();

        // Prevent race condition when writing to the LED.
        xSemaphoreTake(args->ledMutex, portMAX_DELAY);
        // Save original state of LED so it can be restored later on.
        uint32_t ledStates;
        LEDRead(&ledStates);

        // Turn on the LED.
        LEDWrite(args->ledId, args->ledId);

        vTaskDelayUntil(&startTimePoint, sleepDuration);

        // Return the LED to its state before holding.
        LEDWrite(args->ledId, ledStates);

        xSemaphoreGive(args->ledMutex);
        // Allow the handler for the button press to signal a button press again.
        BaseType_t success = xSemaphoreTake(args->buttonSemaphore, 0);
        assert(success); // We should always be able to take the semaphore.
    }
}

void buttonHandler(void *parameters)
{
    const ButtonHandlerArgs *args = parameters;
    const TickType_t sleepDuration = pdMS_TO_TICKS(10);

    while (true)
    {
        uint8_t ucState;
        uint8_t ucDelta;
        // Poll the buttons.
        ucState = ButtonsPoll(&ucDelta, 0);

        if (BUTTON_PRESSED(LEFT_BUTTON, ucState, ucDelta))
        {
            // Turn on LED 1 by releasing the semaphore button_sem_1
            xSemaphoreGive(args->leftButtonSemaphore);
        }

        if (BUTTON_PRESSED(RIGHT_BUTTON, ucState, ucDelta))
        {
            // Turn on LED 2 by releasing the semaphore button_sem_2
            xSemaphoreGive(args->rightButtonSemaphore);
        }

        // We have to have some delay here else we risk starvation.
        vTaskDelay(sleepDuration);
    }
}

int main(void)
{
    // Set the system clock to the same as in FreeRTOSConfig.h.
    SysCtlClockFreqSet(
    SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480,
                       SYSTEM_CLOCK);

    // Create mutexes and binaries
    SemaphoreHandle_t LED_mutex_1 = xSemaphoreCreateMutex();
    SemaphoreHandle_t LED_mutex_2 = xSemaphoreCreateMutex();

    SemaphoreHandle_t button_sem_1 = xSemaphoreCreateBinary();
    SemaphoreHandle_t button_sem_2 = xSemaphoreCreateBinary();

    // Init LEDs
    // Configure the device pins.
    PinoutSet(false, false);

    // Enable the GPIO pin for the LED (PN0,PN1,PF4,PF0).
    // Set the direction as output, and enable the GPIO pin for digital function.
    // LED 1 and 2.
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // LED 3 and 4.
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

    // Initialize the button driver.
    ButtonsInit();

    const uint16_t taskStackDepth = 1000;
    const UBaseType_t blinkTaskPrio = tskIDLE_PRIORITY + 1;
    const UBaseType_t holdTaskPrio = tskIDLE_PRIORITY + 1;
    const UBaseType_t buttonHandlerPrio = tskIDLE_PRIORITY + 1;

    // Construct task arguments as static variables to make sure they outlive the tasks.
    static TaskBlinkArgs blink1Args;
    blink1Args = (TaskBlinkArgs ) { .ledMutex = LED_mutex_1, .blinkDuration =
                                            1000,
                                    .startOffsetSeconds = 0, .ledId = CLP_D1 };
    static TaskBlinkArgs blink2Args;
    blink2Args = (TaskBlinkArgs ) { .ledMutex = LED_mutex_2, .blinkDuration =
                                            2000,
                                    .startOffsetSeconds = 2, .ledId = CLP_D2 };
    static TaskBlinkArgs blink3Args;
    blink3Args = (TaskBlinkArgs ) { .ledMutex = NULL, .blinkDuration = 3000,
                                    .startOffsetSeconds = 4, .ledId = CLP_D3 };
    static TaskBlinkArgs blink4Args;
    blink4Args = (TaskBlinkArgs ) { .ledMutex = NULL, .blinkDuration = 4000,
                                    .startOffsetSeconds = 6, .ledId = CLP_D4 };

    static TaskHoldArgs hold1Args;
    hold1Args = (TaskHoldArgs ) { .ledMutex = LED_mutex_1, .buttonSemaphore =
                                          button_sem_1,
                                  .ledId = CLP_D1 };
    static TaskHoldArgs hold2Args;
    hold2Args = (TaskHoldArgs ) { .ledMutex = LED_mutex_2, .buttonSemaphore =
                                          button_sem_2,
                                  .ledId = CLP_D2 };

    static ButtonHandlerArgs buttonHandlerArgs;
    buttonHandlerArgs = (ButtonHandlerArgs ) { .leftButtonSemaphore =
                                                       button_sem_1,
                                               .rightButtonSemaphore =
                                                       button_sem_2 };

    BaseType_t result;
    result = xTaskCreate(vTaskBlink, "BlinkLed1", taskStackDepth, &blink1Args,
                         blinkTaskPrio, NULL);
    assert(result == pdPASS);

    result = xTaskCreate(vTaskBlink, "BlinkLed2", taskStackDepth, &blink2Args,
                         blinkTaskPrio, NULL);
    assert(result == pdPASS);

    result = xTaskCreate(vTaskBlink, "BlinkLed3", taskStackDepth, &blink3Args,
                         blinkTaskPrio, NULL);
    assert(result == pdPASS);

    result = xTaskCreate(vTaskBlink, "BlinkLed4", taskStackDepth, &blink4Args,
                         blinkTaskPrio, NULL);
    assert(result == pdPASS);

    result = xTaskCreate(vTaskHold, "HoldLed1", taskStackDepth, &hold1Args,
                         holdTaskPrio, NULL);
    assert(result == pdPASS);

    result = xTaskCreate(vTaskHold, "HoldLed2", taskStackDepth, &hold2Args,
<<<<<<< HEAD
                         holdTaskPrio, &hold2Task);
=======
                         holdTaskPrio, NULL);
    assert(result == pdPASS);

    result = xTaskCreate(buttonHandler, "buttonHandler", taskStackDepth,
                         &buttonHandlerArgs, buttonHandlerPrio, NULL);
    assert(result == pdPASS);
>>>>>>> 0fd3454077a24c5da93153ae28d99b69fdfc1beb

    // Start the scheduler with vTaskStartScheduler
    vTaskStartScheduler();

    // We should never reach here since vTaskStartScheduler should block.
    assert(false);
    while (true)
    {
    }
}

