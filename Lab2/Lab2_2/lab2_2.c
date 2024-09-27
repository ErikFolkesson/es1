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
    LED_PWM_OUT = PWM_OUT_2, // The pin for the LED uses PWM output 2.
    LED_PWM_OUT_BIT = PWM_OUT_2_BIT,
    LED_PWM_GENERATOR = PWM_GEN_1, // PWM output 2 is associated with generator 1.
    LED_PWM_PIN_CONFIGURATION = GPIO_PF2_M0PWM2,
    JOY_ADC_SEQ_NUM = 3, // Sequence 3 only samples one value.
    JOY_ADC_BASE = ADC0_BASE,
};

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

// Initializes the necessary ports/pins and sets up an ADC sequencer (JOY_ADC_SEQ_NUM) ready for sampling.
void joystickSetup()
{
    enum
    {
        ADC_HOR_JOY_CH = ADC_CTL_CH9, // The pin for horizontal movement (PE4) is connected to channel 9 (AIN9).
        ADC_HIGHEST_PRIO = 0,
        ADC_PERIPH = SYSCTL_PERIPH_ADC0,
        ADC_GPIO_PORT_PERIPH = SYSCTL_PERIPH_GPIOE,
        ADC_GPIO_PORT_BASE = GPIO_PORTE_BASE,
        ADC_GPIO_PIN = GPIO_PIN_4,
        ADC_STEP = 0, // For JOY_ADC_SEQ_NUM = 3, the only valid step is step 0.
    };

    // Enable the ADC peripheral port
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

    // ADC_TRIGGER_PROCESSOR is used to allows us to trigger the reading of the joystick using the ADCProcessorTrigger function.
    ADCSequenceConfigure(JOY_ADC_BASE, JOY_ADC_SEQ_NUM, ADC_TRIGGER_PROCESSOR,
                         ADC_HIGHEST_PRIO);

    // ADC_CTL_END: This step is the last in the sequence.
    // ADC_CTL_IE: When this step is complete, it will cause an interrupt.
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

    // Wait until the interrupt sampling the joystick value has completed.
    while (ADC_READ_VALUE_INTERRUPT
            & ADCIntStatus(JOY_ADC_BASE, JOY_ADC_SEQ_NUM, true))
    {
    }

    uint32_t buffer;

    int32_t samplesRead = ADCSequenceDataGet(JOY_ADC_BASE, JOY_ADC_SEQ_NUM,
                                             &buffer); // 0 lowest | 1450-1950 middle | 4000 higher
    assert(samplesRead == 1);

    return buffer;
}

// Reads the current value of the joystick and sets the brightness based on the value.
void setBrightnessFromJoystick()
{

    uint32_t joystickValue = readJoystick();
    // We divide by 40 since we want to generate a value between [0,100], and the maximum value that readJoystick can return is around 4000.
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

    joystickSetup();

    while (1)
    {
        setBrightnessFromJoystick();
    }
}
