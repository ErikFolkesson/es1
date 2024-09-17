#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "drivers/buttons.h"
#include "drivers/pinout.h"


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
    while(1) {
        // Poll the buttons.
        ucState = ButtonsPoll(&ucDelta, 0);
        if(BUTTON_PRESSED(RIGHT_BUTTON, ucState, ucDelta)) {
            while (true) {
                // Turn off D1.
                LEDWrite(CLP_D1, CLP_D1);

                for (i = 0; i < BLINK_SPEED; i++) {
                }

                LEDWrite(CLP_D1, 0);

                for (i = 0; i < BLINK_SPEED; i++) {
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
    while(1) {
        // Poll the buttons.
        ucState = ButtonsPoll(&ucDelta, 0);

        if(BUTTON_PRESSED(RIGHT_BUTTON, ucState, ucDelta)) {
            LEDWrite(CLP_D1, CLP_D1);
            LED_ON = true;
        } else if (BUTTON_RELEASED(RIGHT_BUTTON, ucState, ucDelta)){
            LEDWrite(CLP_D1, 0);
            LED_ON = false;
        }

        if(BUTTON_PRESSED(LEFT_BUTTON, ucState, ucDelta)) {
            if (!LED_ON){
                LEDWrite(CLP_D1, CLP_D1);
                LED_ON = true;
            } else {
                LEDWrite(CLP_D1, 0);
                LED_ON = false;
            }
        }
    }
}

void optionalAssig()
{
    unsigned char ucDelta, ucState;
    volatile int i = 0;
    volatile int j = 0;

    // Configure the device pins.
    PinoutSet(false, false);

    // Initialize the button driver.
    ButtonsInit();

    // Enable the GPIO pin for the LED (PN0 - PN3).
    // Set the direction as output, and
    // enable the GPIO pin for digital function.
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    // Loop forever.
    while(1) {
        // Poll the buttons.
        ucState = ButtonsPoll(&ucDelta, 0);

        if (BUTTON_PRESSED(RIGHT_BUTTON, ucState, ucDelta)) {
            while (true) {
                // Array to store the LED pin definitions
                unsigned char leds[] = {CLP_D1, CLP_D2, CLP_D3, CLP_D4};

                // Automatically calculate the number of objects in leds[]
                // Allows for easier adding or removal of LEDs.
                int num_leds = sizeof(leds) / sizeof(leds[0]);

                // Iterate over each LED
                for (j = 0; j < num_leds; j++) {
                    // Turn on the current LED
                    LEDWrite(leds[j], leds[j]);

                    // Wait for a quarter of the blink speed
                    for (i = 0; i < BLINK_SPEED / 4; i++) {}

                    // Turn off the current LED
                    LEDWrite(leds[j], 0);
                }

                // Iterate back the other way
                // j starts on 2 so that it doesn't light up LED 4 two times.
                for (j = 2; j > 0; j--) {
                    // Turn on the current LED
                    LEDWrite(leds[j], leds[j]);

                    // Wait for a quarter of the blink speed
                    for (i = 0; i < BLINK_SPEED / 4; i++) {}

                    // Turn off the current LED
                    LEDWrite(leds[j], 0);
                }
            }
        }

    }
}

// Main Function
int main(void)
{
    // assig1();
    // assig2();
    optionalAssig();
}
