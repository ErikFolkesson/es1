#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <inc/hw_memmap.h>
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "drivers/buttons.h"
#include "drivers/pinout.h"

#define ARR_SIZE(arr) (sizeof(arr) / sizeof(*(arr)))

static const int ledSequence[] = { CLP_D1, CLP_D2, CLP_D3, CLP_D4, CLP_D3,
CLP_D2 };

// Runs through an empty loop for a while to delay execution.
static void DelayExecution()
{
    enum
    {
        delayLoopIterations = 200000
    };
    volatile int i;
    for (i = 0; i < delayLoopIterations; ++i)
    {
        // Intentionally empty.
    }
}

int Lab1OptionalJohan(void)
{
    bool reserveEthernetLed = false;
    bool reserveUsbLed = false;
    int currentLedIndex;
    PinoutSet(reserveEthernetLed, reserveUsbLed);

    // Enable GPIO pin PN0 as an output pin to use for the LED.
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);

    while (true)
    {
        // We handle the initial LED as a special case (since we don't need to turn
        // any other LED off.
        LEDWrite(ledSequence[0], 1);

        // Allow the LED to shine for a bit.
        DelayExecution();

        // We start at index 1 since the first LED has already been handled.
        for (currentLedIndex = 1; currentLedIndex < ARR_SIZE(ledSequence);
                ++currentLedIndex)
        {
            // Turn off the previous LED and turn on the new LED.
            LEDWrite(ledSequence[currentLedIndex], 1);
            LEDWrite(ledSequence[currentLedIndex - 1], 0);

            // Allow the LED to shine for a bit.
            DelayExecution();
        }

        // We end the sequence by turning off the last LED, allowing for the
        // sequence to restart from a fresh state.
        LEDWrite(ledSequence[ARR_SIZE(ledSequence) - 1], 0);
    }
}
