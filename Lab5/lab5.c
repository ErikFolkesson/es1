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

typedef struct
{
    SemaphoreHandle_t *ledMutex; // The mutex for modifying/holding the LED.
    uint16_t blinkDuration; // How many milliseconds the LED should spend in each on/off state.
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

    // FixMe: Maybe add synch?

    // Get start time
    TickType_t currentWakeTime = xTaskGetTickCount();

    while (true)
    {
        if (args->ledMutex == NULL)
        {
            // Turn on LED args.ledId
            LEDWrite((args->ledId), (args->ledId));

            // Wait for args.blinkDuration m
            vTaskDelayUntil(&currentWakeTime,
                            args->blinkDuration / portTICK_PERIOD_MS);

            // Turn off LED args.ledId
            LEDWrite((args->ledId), 0);

            // Wait for args.blinkDuration ms
            vTaskDelayUntil(&currentWakeTime,
                            args->blinkDuration / portTICK_PERIOD_MS);

        }
        else
        {
            // take mutex without blocking
            BaseType_t success = xSemaphoreTake(args->ledMutex, 0);

            if (success)
            {
                // Turn on LED args.ledId
                LEDWrite((args->ledId), (args->ledId));
            }

            // Wait for args.blinkDuration m
            vTaskDelayUntil(&currentWakeTime,
                            args->blinkDuration / portTICK_PERIOD_MS);

            if (success)
            {
                // Turn off LED args.ledId
                LEDWrite((args->ledId), 0);
            }

            // Wait for args.blinkDuration ms
            vTaskDelayUntil(&currentWakeTime,
                            args->blinkDuration / portTICK_PERIOD_MS);
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

// FixMe: Interrupt handler for the buttons?

int main(void)
{
    // Create mutexes and binaries
    SemaphoreHandle_t LED_mutex_1 = xSemaphoreCreateMutex();
    SemaphoreHandle_t LED_mutex_2 = xSemaphoreCreateMutex();

    SemaphoreHandle_t button_sem_1 = xSemaphoreCreateCounting(1, 0);
    SemaphoreHandle_t button_sem_2 = xSemaphoreCreateCounting(1, 0);

    // FixMe: Block for 10 ms
    const TickType_t xDelay = 10 / portTICK_PERIOD_MS;

    // Init LEDs
    // Configure the device pins.
    PinoutSet(false, false);

    // Enable the GPIO pin for the LED (PN0 - PN3).
    // Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE,
    GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    // Initialize the button driver.
    ButtonsInit();
    unsigned char ucDelta, ucState;

    // FIXME: These could be defines? Potentially enums.
    const uint16_t taskStackDepth = 1000; // FIXME: Figure out good stack depth. (shouldn't need to be large?)
    const UBaseType_t blinkTaskPrio = tskIDLE_PRIORITY + 2;
    const UBaseType_t holdTaskPrio = tskIDLE_PRIORITY + 1;

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

    // FIXME: Might not need the handles?
    TaskHandle_t blink1Task;
    BaseType_t result = xTaskCreate(vTaskBlink, "BlinkLed1", taskStackDepth,
                                    &blink1Args, blinkTaskPrio, &blink1Task);
    assert(result == pdPASS);
    // FIXME: In the example used in the xTaskCreate doc comment, they use configASSERT(taskHandle). Figure out what that actually does.

    TaskHandle_t blink2Task;
    result = xTaskCreate(vTaskBlink, "BlinkLed2", taskStackDepth, &blink2Args,
                         blinkTaskPrio, &blink2Task);
    assert(result == pdPASS);

    TaskHandle_t blink3Task;
    result = xTaskCreate(vTaskBlink, "BlinkLed3", taskStackDepth, &blink3Args,
                         blinkTaskPrio, &blink3Task);
    assert(result == pdPASS);

    TaskHandle_t blink4Task;
    result = xTaskCreate(vTaskBlink, "BlinkLed4", taskStackDepth, &blink4Args,
                         blinkTaskPrio, &blink4Task);
    assert(result == pdPASS);

    TaskHandle_t hold1Task;
    result = xTaskCreate(vTaskHold, "HoldLed1", taskStackDepth, &hold1Args,
                         holdTaskPrio, &hold1Task);
    assert(result == pdPASS);

    TaskHandle_t hold2Task;
    result = xTaskCreate(vTaskHold, "HoldLed2", taskStackDepth, &hold2Args,
                         holdTaskPrio, &hold2Task);
    assert(result == pdPASS);

    // Start the scheduler with vTaskStartScheduler
    vTaskStartScheduler();

    while (true)
    {
        // Poll the buttons.
        ucState = ButtonsPoll(&ucDelta, 0);

        if (BUTTON_PRESSED(RIGHT_BUTTON, ucState, ucDelta))
        {
            // Turn on LED 1 by releasing the semaphore button_sem_1
            xSemaphoreGive(button_sem_1);
        }

        if (BUTTON_PRESSED(LEFT_BUTTON, ucState, ucDelta))
        {
            // Turn on LED 2 by releasing the semaphore button_sem_2
            xSemaphoreGive(button_sem_2);
        }

        // We have to have some delay here else we risk starvation.
        vTaskDelay(xDelay);
    }
}

