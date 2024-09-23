#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "drivers/buttons.h"
#include "drivers/pinout.h"

#include "optional_erik.h"
#include "johan_optional.h"

// The error routine that is called if the driver library
// encounters an error.
#ifdef DEBUG

void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif

#define BLINK_SPEED 600000

void assig1()
{
    unsigned char ucDelta, ucState;
    volatile int i = 0;

    // Configure the device pins.
    PinoutSet(false, false);

    // Initialize the button driver.
    ButtonsInit();

    // Enable the GPIO pin for the LED (PN0).
    // Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);

    // Loop forever.
    while (1)
    {
        // Poll the buttons.
        ucState = ButtonsPoll(&ucDelta, 0);
        if (BUTTON_PRESSED(RIGHT_BUTTON, ucState, ucDelta))
        {
            while (true)
            {
                // Turn off D1.
                LEDWrite(CLP_D1, CLP_D1);

                for (i = 0; i < BLINK_SPEED; i++)
                {
                }

                LEDWrite(CLP_D1, 0);

                for (i = 0; i < BLINK_SPEED; i++)
                {
                }
            }
        }
    }
}

void assig2()
{
    unsigned char ucDelta, ucState;
    bool LED_ON = false;

    // Configure the device pins.
    PinoutSet(false, false);

    // Initialize the button driver.
    ButtonsInit();

    // Enable the GPIO pin for the LED (PN0).
    // Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);

    // Loop forever.
    while (1)
    {
        // Poll the buttons.
        ucState = ButtonsPoll(&ucDelta, 0);

        if (BUTTON_PRESSED(RIGHT_BUTTON, ucState, ucDelta))
        {
            LEDWrite(CLP_D1, CLP_D1);
            LED_ON = true;
        }
        else if (BUTTON_RELEASED(RIGHT_BUTTON, ucState, ucDelta))
        {
            LEDWrite(CLP_D1, 0);
            LED_ON = false;
        }

        if (BUTTON_PRESSED(LEFT_BUTTON, ucState, ucDelta))
        {
            if (!LED_ON)
            {
                LEDWrite(CLP_D1, CLP_D1);
                LED_ON = true;
            }
            else
            {
                LEDWrite(CLP_D1, 0);
                LED_ON = false;
            }
        }
    }
}

// Main Function
int main(void)
{
    // assig1();
    // assig2();
    optionalAssigErik();
    // Lab1OptionalJohan();
}
