//*****************************************************************************
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
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
    LED_PWM_OUT = PWM_OUT_2,
    LED_PWM_OUT_BIT = PWM_OUT_2_BIT,
    LED_PWM_GENERATOR = PWM_GEN_1,
    LED_PWM_PIN_CONFIGURATION = GPIO_PF2_M0PWM2,
    JOY_ADC_SEQ_NUM = 3, // Sequence 3 only reads one value.
    JOY_ADC_BASE = ADC0_BASE,
};

// desiredBrightness should be integer in range 0 - 100, if the value is outside the range it will be clamped.
// Sets the brightness of the red LED.
void setBrightness(int desiredBrightness)
{
    if (desiredBrightness >= 100)
    {
        // We want to set the pin to the highest possible value
        PWMPulseWidthSet(LED_PWM_BASE, LED_PWM_OUT, pwm_word / 1); // Strongest = pwm_word/1 | Weakest = pwm_word/10000
        PWMOutputState(LED_PWM_BASE, LED_PWM_OUT_BIT, true);

    }
    else if (desiredBrightness <= 0)
    {
        // We want to turn off the lamp
        PWMPulseWidthSet(LED_PWM_BASE, LED_PWM_OUT, pwm_word / 10000);
        PWMOutputState(LED_PWM_BASE, LED_PWM_OUT_BIT, false);

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
        PWMPulseWidthSet(LED_PWM_BASE, LED_PWM_OUT, width);
        PWMOutputState(LED_PWM_BASE, LED_PWM_OUT_BIT, true);
    }
}

void joystickSetup()
{
    enum
    {
        ADC_HOR_JOY_CH = ADC_CTL_CH9, // AIN9 in table 2-1 in the user guide for tm4c129exl.
        ADC_HIGHEST_PRIO = 0,
        ADC_PERIPH = SYSCTL_PERIPH_ADC0,
        ADC_GPIO_PORT_PERIPH = SYSCTL_PERIPH_GPIOE,
        ADC_GPIO_PORT_BASE = GPIO_PORTE_BASE,
        ADC_GPIO_PIN = GPIO_PIN_4,
        ADC_STEP = 0, // For JOY_ADC_SEQ_NUM = 3, the only valid step is step 0.
    };

    // Enable he ADC peripheral port
    SysCtlPeripheralEnable(ADC_PERIPH);

    while (!SysCtlPeripheralReady(ADC_PERIPH))
    {
    }

    // Enable the GPIO port that is associated with the peripheral (joystick)
    SysCtlPeripheralEnable(ADC_GPIO_PORT_PERIPH);

    while (!SysCtlPeripheralReady(ADC_GPIO_PORT_PERIPH))
    {
    }

    GPIOPinTypeADC(ADC_GPIO_PORT_BASE, ADC_GPIO_PIN);

    // ADC_TRIGGER_PROCESSOR is used to allows us to trigger the reading of the joystick
    // SEQ_NUM = 3 because we only care about one value when reading the ADC pin.
    // Highest priority
    ADCSequenceConfigure(JOY_ADC_BASE, JOY_ADC_SEQ_NUM, ADC_TRIGGER_PROCESSOR,
                         ADC_HIGHEST_PRIO);
    ADCSequenceStepConfigure(JOY_ADC_BASE, JOY_ADC_SEQ_NUM, ADC_STEP,
                             ADC_HOR_JOY_CH | ADC_CTL_END | ADC_CTL_IE);

    ADCSequenceEnable(JOY_ADC_BASE, JOY_ADC_SEQ_NUM);
}

// Reads the current value of the joystick. The value will be in about in the range [0, 4000].
uint32_t readJoystick()
{
    enum
    {
        ADC_READ_VALUE_INTERRUPT = ADC_INT_SS3, // SS3 = sample sequence 3.
    };
    ADCProcessorTrigger(JOY_ADC_BASE, JOY_ADC_SEQ_NUM);

    // Wait until the interrupt is handled
    while (ADC_READ_VALUE_INTERRUPT
            & ADCIntStatus(JOY_ADC_BASE, JOY_ADC_SEQ_NUM, true))
    {
    }

    uint32_t buffer;

    int32_t samplesRead = ADCSequenceDataGet(JOY_ADC_BASE, JOY_ADC_SEQ_NUM, &buffer); // 0 lowest | 1450-1950 middle | 4000 higher
    assert(samplesRead == 1);

    return buffer;
}

// Reads the current value of the joystick and sets the brightness based on the value.
void setBrightnessFromJoystick()
{

    uint32_t joystickValue = readJoystick();
    int desiredBrightness = joystickValue / 40;

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

    // These can be removed. Maybe because it could be enabled from the beginning?
    // SysCtlPeripheralDisable(SYSCTL_PERIPH_PWM0);
    // SysCtlPeripheralReset(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(LED_PWM_PERIPH);

    GPIOPinTypePWM(LED_GPIO_BASE, LED_GPIO_PIN);
    GPIOPinConfigure(LED_PWM_PIN_CONFIGURATION);

    PWMGenConfigure(LED_PWM_BASE, LED_PWM_GENERATOR,
    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC | PWM_GEN_MODE_DBG_RUN);

    PWMGenPeriodSet(LED_PWM_BASE, LED_PWM_GENERATOR, pwm_word);

    PWMPulseWidthSet(LED_PWM_BASE, LED_PWM_OUT, pwm_word / 10000); // Strongest = pwm_word/1 | Weakest = pwm_word/10000

    PWMGenEnable(LED_PWM_BASE, LED_PWM_GENERATOR);

    PWMOutputState(LED_PWM_BASE, LED_PWM_OUT_BIT, true);

    joystickSetup();

    while (1)
    {
        setBrightnessFromJoystick();
    }
}
