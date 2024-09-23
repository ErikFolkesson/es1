//*****************************************************************************
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "inc/tm4c129encpdt.h"

#include <math.h>
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

#include <stdlib.h>

#include "driverlib/adc.h"

#include "drivers/buttons.h"
#include "drivers/pinout.h"

float pwm_word;
uint32_t systemClock;

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

//***********************************************************************
//                       Configurations
//***********************************************************************
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

// desiredBrightness should be integer in range 0 - 100, if the value is outside the range it will be clamped.
// Sets the brightness of the red LED.
void setBrightness(int desiredBrightness)
{
    if (desiredBrightness >= 100)
    {
        // We want to set the pin to the highest possible value
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pwm_word / 1); // Strongest = pwm_word/1 | Weakest = pwm_word/10000
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);

    }
    else if (desiredBrightness <= 0)
    {
        // We want to turn off the lamp
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pwm_word / 10000);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, false);

    }
    else
    {
        // Since the brightness seems to be logarithmic base 2, we want to find the inverse so that we can scale it to become roughly linear from 0 to 100.
        // We found that the brightness seemed to be lowest around pwm_word * (1 / 10'000).
        // We then want to find x in 2^((desiredBrightness-100)/x) = 1 / 10'000, when desiredBrightness = 0 and x is some scaling factor.
        double scalingFactor = 7.5;
        float exponent = ((float) desiredBrightness - 100.0) / scalingFactor;

        // The brightness should be somewhere between min and max brightness.
        float width = pwm_word * pow(2, exponent); // Strongest = pwm_word/1 | Weakest = pwm_word/10000
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, width);
        PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
    }
}

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void UARTIntHandler(void)
{
    uint32_t ui32Status;

    // Get the interrupt status.
    ui32Status = MAP_UARTIntStatus(UART0_BASE, true);

    // Clear the asserted interrupts.
    MAP_UARTIntClear(UART0_BASE, ui32Status);

    // Loop while there are characters in the receive FIFO.
    while (MAP_UARTCharsAvail(UART0_BASE))
    {
        enum
        {
            arrSize = 10
        };
        char buf[arrSize];

        // If I understand this function correctly, it will read the input until a newline character is found.
        // Meaning that the while loop we are in will only run once.
        // This is why we can recivie multiple characters in one go.
        // `UARTgets` contains it's own loop that reads characters until a newline character is found.
        UARTgets(buf, arrSize);

        // All non integer values becomes 0.
        int desiredBrightness = atoi(buf);

        setBrightness(desiredBrightness);

    }
}

//*****************************************************************************
//
//                      Main
//
//*****************************************************************************
int main(void)
{
    ConfigureUART();

    systemClock = SysCtlClockFreqSet(
            (SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL
                    | SYSCTL_CFG_VCO_480),
            16000);

// Enable processor interrupts.
    MAP_IntMasterEnable();

// Register the interrupt handler function for UART 0.
    IntRegister(INT_UART0, UARTIntHandler);

// Enable the UART interrupt.
    MAP_IntEnable(INT_UART0);
    MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    pwm_word = systemClock / 200;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);

    PWMGenConfigure(
            PWM0_BASE, PWM_GEN_1,
            PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_RUN);

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, pwm_word);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pwm_word / 10000); // Strongest = pwm_word/1 | Weakest = pwm_word/10000

    PWMGenEnable(PWM0_BASE, PWM_GEN_1);

    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);

    UARTprintf("\033[2JEnter text: ");

    assig1();
}
