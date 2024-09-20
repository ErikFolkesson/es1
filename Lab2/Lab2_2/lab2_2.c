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

#define SEQ_NUM 3

void joystickSetup()
{
    // Enable he ADC peripheral port
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    }

    // Enable the GPIO port that is associated with the peripheral (joystick)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
    {
    }

    // Sets pin 4 as ADC pin
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4);
    GPIOPinConfigure(GPIO_PE4_U1RI);

    // ADC_TRIGGER_PROCESSOR is used to allows us to trigger the reading of the joystick
    // SEQ_NUM = 3 because we only care about one value when reading the ADC pin.
    // Highest priority
    ADCSequenceConfigure(ADC0_BASE, SEQ_NUM, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, SEQ_NUM, 0, ADC_CTL_END | ADC_CTL_IE);

    ADCSequenceEnable(ADC0_BASE, SEQ_NUM);
}

void joystick()
{
    ADCProcessorTrigger(ADC0_BASE, SEQ_NUM);

    // Wait until the interrupt is handled
    while (ADC_INT_SS0 & ADCIntStatus(ADC0_BASE, SEQ_NUM, true))
    {
    }

    uint32_t buffer;

    ADCSequenceDataGet(ADC0_BASE, SEQ_NUM, &buffer); // 0 lowest | 1450-1950 middle | 4000 higher

    int desiredBrightness = buffer / 40;

    setBrightness(desiredBrightness);
}

//*****************************************************************************
//
//                      Main
//
//*****************************************************************************
int main(void)
{
    systemClock = SysCtlClockFreqSet(
            (SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL
                    | SYSCTL_CFG_VCO_480),
            16000);

    pwm_word = systemClock / 200;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    SysCtlPeripheralDisable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_PWM0);
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

    joystickSetup();

    while (1)
    {
        joystick();
    }
}
