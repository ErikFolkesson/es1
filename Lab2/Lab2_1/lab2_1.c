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

float pwm_word;
uint32_t systemClock;

// Constants
enum
{
    LED_GPIO_BASE = GPIO_PORTF_BASE,
    LED_GPIO_PIN = GPIO_PIN_2,
    LED_PWM_PERIPH = SYSCTL_PERIPH_PWM0,
    LED_PWM_BASE = PWM0_BASE,
    LED_PWM_OUT = PWM_OUT_2, // The pin for the LED uses PWM output 2.
    LED_PWM_OUT_BIT = PWM_OUT_2_BIT,
    LED_PWM_GENERATOR = PWM_GEN_1, // PWM output 2 is associated with generator 1.
    LED_PWM_PIN_CONFIGURATION = GPIO_PF2_M0PWM2,
};

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

// Makes sure that the PWM peripheral is enabled and that the port and pin for the red LED are configured as using PWM.
void enablePwm(void)
{
    // If PWM is already enabled, we don't need to do anything.
    if (SysCtlPeripheralReady(LED_PWM_PERIPH))
    {
        return;
    }

    SysCtlPeripheralEnable(LED_PWM_PERIPH);
    while (!SysCtlPeripheralReady(LED_PWM_PERIPH))
    {
    }
    GPIOPinTypePWM(LED_GPIO_BASE, LED_GPIO_PIN);
    GPIOPinConfigure(LED_PWM_PIN_CONFIGURATION);
}

// Makes sure that the GPIO peripheral is enabled and that the port and pin for the red LED are configured as using GPIO output.
void enableGpio(void)
{
    // If PWM is enabled, we need to disable it.
    if (SysCtlPeripheralReady(LED_PWM_PERIPH))
    {
        SysCtlPeripheralDisable(LED_PWM_PERIPH);
    }

    GPIOPinTypeGPIOOutput(LED_GPIO_BASE, LED_GPIO_PIN);
}

// desiredBrightness should be integer in range 0 - 100, if the value is outside the range it will be clamped.
// Sets the brightness of the red LED.
void setBrightness(int desiredBrightness)
{
    if (desiredBrightness >= 100)
    {
        enableGpio();
        GPIOPinWrite(LED_GPIO_BASE, LED_GPIO_PIN, LED_GPIO_PIN);
    }
    else if (desiredBrightness <= 0)
    {
        enableGpio();
        GPIOPinWrite(LED_GPIO_BASE, LED_GPIO_PIN, 0);
    }
    else
    {
        enablePwm();

        // Since the brightness seems to be logarithmic base 2, we want to find the inverse so that we can scale it to become roughly linear from 0 to 100.
        // We found that the brightness seemed to be lowest around pwm_word * (1 / 10'000).
        // We then want to find x in 2^((desiredBrightness-100)/x) = 1 / 10'000, when desiredBrightness = 0 and x is some scaling factor.
        double scalingFactor = 7.5;
        float exponent = ((float) desiredBrightness - 100.0) / scalingFactor;

        // The brightness should be somewhere between min and max brightness.
        float width = pwm_word * pow(2, exponent); // Strongest = pwm_word/1 | Weakest = pwm_word/10000
        PWMPulseWidthSet(LED_PWM_BASE, LED_PWM_OUT, width);
        PWMOutputState(LED_PWM_BASE, LED_PWM_OUT_BIT, true);
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

    // Enable the UART interrupt. RX = Receive, RT = Receive timeout.
    MAP_IntEnable(INT_UART0);
    MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    pwm_word = systemClock / 200;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    SysCtlPeripheralEnable(LED_PWM_PERIPH);

    GPIOPinTypePWM(LED_GPIO_BASE, LED_GPIO_PIN);
    GPIOPinConfigure(LED_PWM_PIN_CONFIGURATION);

    // PWM_GEN_MODE_DOWN: counts from some value down to 0 and then resets, "producing left-aligned PWM signals".
    // PWM_GEN_MODE_NO_SYNC: Any changes to period or pulse width will be applied immediately the next time the counter becomes 0 instead of waiting for a synchronization event.
    // PWM_GEN_MODE_DBG_RUN: Don't pause the PWM while the process is stopped in the debugger.
    PWMGenConfigure(LED_PWM_BASE, LED_PWM_GENERATOR,
    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_RUN);

    PWMGenPeriodSet(LED_PWM_BASE, LED_PWM_GENERATOR, pwm_word);

    PWMPulseWidthSet(LED_PWM_BASE, LED_PWM_OUT, pwm_word / 10000); // Strongest = pwm_word/1 | Weakest = pwm_word/10000

    PWMGenEnable(LED_PWM_BASE, LED_PWM_GENERATOR);

    PWMOutputState(LED_PWM_BASE, LED_PWM_OUT_BIT, true);

    UARTprintf("\033[2JEnter text: ");

    while (1)
    {
    }
}
