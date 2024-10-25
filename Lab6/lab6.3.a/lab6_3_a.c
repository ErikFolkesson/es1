#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <ctype.h>
#include <stdio.h>

#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "drivers/buttons.h"

#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "inc/tm4c129encpdt.h"
#include "driverlib/adc.h"

void ConfigureUART(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    uint32_t baudRate = 115200;
    // 8 data bits, 1 stop bit, and no parity bit.
    uint32_t uartConfig = UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE
            | UART_CONFIG_PAR_NONE;
    // For PIOSC clock, the frequency should be set as 16'000'000
    uint32_t systemClockFrequency = 16000000;
    UARTConfigSetExpClk(UART0_BASE, systemClockFrequency, baudRate, uartConfig);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    UARTEnable(UART0_BASE);
}

void uartPuts(const char *str)
{
    const char *it;
    for (it = str; *it != '\0'; it++)
    {
        UARTCharPut(UART0_BASE, *it);
    }
}

// Prints the ANSI escape sequence to move the cursor to the home position (1,1)
void clearScreenAndMoveCursorHome(void)
{
    uartPuts("\x1B[2J");
    uartPuts("\x1B[H");
}

// NOTE: We don't handle numbers larger than 99.
void uartPrintInt(int number)
{
    char tenDigit = '0' + number / 10 % 10;
    char oneDigit = '0' + number % 10;
    UARTCharPut(UART0_BASE, tenDigit);
    UARTCharPut(UART0_BASE, oneDigit);
}

enum
{
    ADC_JOY_SEQ_NUM = 2, // Sequence 2 samples four values.

    ADC_MIC_SEQ_NUM = 3, // Sequence 3 samples 1 values.

    ADC_ACCEL_SEQ_NUM = 1, // Sequence 1 sampler four values.

    ADC_BASE = ADC0_BASE,

    ADC_HOR_JOY_CH = ADC_CTL_CH9, // The pin for horizontal movement (PE4) is connected to channel 9 (AIN9).
    ADC_VER_JOY_CH = ADC_CTL_CH0, // Same logic as above.
    ADC_ACCEL_X_CH = ADC_CTL_CH3,
    ADC_ACCEL_Y_CH = ADC_CTL_CH2,
    ADC_ACCEL_Z_CH = ADC_CTL_CH1,
    ADC_MICROPHONE_CH = ADC_CTL_CH8,

    ADC_HIGHEST_PRIO = 0,
    ADC_PERIPH = SYSCTL_PERIPH_ADC0,
    ADC_GPIO_PORT_PERIPH = SYSCTL_PERIPH_GPIOE,
    ADC_GPIO_PORT_BASE = GPIO_PORTE_BASE,

    ADC_GPIO_JOY_HORIZONTAL_PIN = GPIO_PIN_4,
    ADC_GPIO_JOY_VERTICAL_PIN = GPIO_PIN_3,
    ADC_GPIO_ACCEL_X = GPIO_PIN_0,
    ADC_GPIO_ACCEL_Y = GPIO_PIN_1,
    ADC_GPIO_ACCEL_Z = GPIO_PIN_2,
    ADC_GPIO_MICROPHONE = GPIO_PIN_5,
};

// Initializes the necessary ports/pins and sets up an ADC sequencer (JOY_ADC_SEQ_NUM) ready for sampling.
void ADCSetup()
{
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

    uint32_t pins = ADC_GPIO_JOY_HORIZONTAL_PIN | ADC_GPIO_JOY_VERTICAL_PIN
            | ADC_GPIO_ACCEL_X | ADC_GPIO_ACCEL_Y | ADC_GPIO_ACCEL_Z
            | ADC_GPIO_MICROPHONE;
    GPIOPinTypeADC(ADC_GPIO_PORT_BASE, pins);

    // Turn on the different sequencers we use.
    // ADC_TRIGGER_PROCESSOR is used to allows us to trigger the reading of the peripheral using the ADCProcessorTrigger function.
    ADCSequenceConfigure(ADC_BASE, ADC_JOY_SEQ_NUM, ADC_TRIGGER_PROCESSOR,
                         ADC_HIGHEST_PRIO);
    ADCSequenceConfigure(ADC_BASE, ADC_MIC_SEQ_NUM, ADC_TRIGGER_PROCESSOR,
                         ADC_HIGHEST_PRIO);
    ADCSequenceConfigure(ADC_BASE, ADC_ACCEL_SEQ_NUM, ADC_TRIGGER_PROCESSOR,
                         ADC_HIGHEST_PRIO);

    // Configure steps for both vertical and horizontal joystick.
    // ADC_CTL_END: This step is the last in the sequence.
    // ADC_CTL_IE: When this step is complete, it will cause an interrupt.
    ADCSequenceStepConfigure(ADC_BASE, ADC_JOY_SEQ_NUM, 0, ADC_HOR_JOY_CH);
    ADCSequenceStepConfigure(ADC_BASE, ADC_JOY_SEQ_NUM, 1,
                             ADC_VER_JOY_CH | ADC_CTL_END | ADC_CTL_IE);

    // Configure steps for the accelerometer's x, y, and z.
    // ADC_CTL_END: This step is the last in the sequence.
    // ADC_CTL_IE: When this step is complete, it will cause an interrupt.
    ADCSequenceStepConfigure(ADC_BASE, ADC_ACCEL_SEQ_NUM, 0, ADC_ACCEL_X_CH);
    ADCSequenceStepConfigure(ADC_BASE, ADC_ACCEL_SEQ_NUM, 1, ADC_ACCEL_Y_CH);
    ADCSequenceStepConfigure(ADC_BASE, ADC_ACCEL_SEQ_NUM, 2,
                             ADC_ACCEL_Z_CH | ADC_CTL_END | ADC_CTL_IE);

    // Configure a step for the microphone.
    // ADC_CTL_END: This step is the last in the sequence.
    // ADC_CTL_IE: When this step is complete, it will cause an interrupt.
    ADCSequenceStepConfigure(ADC_BASE, ADC_MIC_SEQ_NUM, 0,
                             ADC_MICROPHONE_CH | ADC_CTL_END | ADC_CTL_IE);

    ADCSequenceEnable(ADC_BASE, ADC_JOY_SEQ_NUM);
    ADCSequenceEnable(ADC_BASE, ADC_ACCEL_SEQ_NUM);
    ADCSequenceEnable(ADC_BASE, ADC_MIC_SEQ_NUM);
}

typedef struct
{
    uint32_t horizontal;
    uint32_t vertical;
} JoystickReading;

// Reads the current value of the joystick. The value will be in about in the range [0, 4000].
JoystickReading readJoystick()
{
    enum
    {
        ADC_READ_VALUE_INTERRUPT = ADC_INT_SS2, // SS2 = sample sequence 2, can read up to 4 samples.
    };

    int32_t samplesRead;
    uint32_t buffer[4] = { 0 };
    do
    {
        ADCProcessorTrigger(ADC_BASE, ADC_JOY_SEQ_NUM);

        // Wait until the interrupt sampling the joystick value has completed.
        while (ADC_READ_VALUE_INTERRUPT
                & ADCIntStatus(ADC_BASE, ADC_JOY_SEQ_NUM, true))
        {
        }

        samplesRead = ADCSequenceDataGet(ADC_BASE, ADC_JOY_SEQ_NUM, buffer); // 0 lowest | 1450-1950 middle | 4000 higher
        assert(samplesRead >= 0 && samplesRead <= 4);
    }
    while (samplesRead < 2);
    // The above do-while loop technically discards legitimate readings, but doesn't require us to implement a queue.
    // It also assumes that whenever two sampler are read, the first is the horizontal value and the second is the vertical.

    JoystickReading ret = { .horizontal = buffer[0], .vertical = buffer[1] };
    return ret;
}

// Reads the current value of the microphone.
uint32_t readMicrophone()
{
    enum
    {
        ADC_READ_VALUE_INTERRUPT = ADC_INT_SS3, // SS3 = sample sequence 3, can read a single sample.
    };

    ADCProcessorTrigger(ADC_BASE, ADC_MIC_SEQ_NUM);

    // Wait until the interrupt sampling the microphone value has completed.
    while (ADC_READ_VALUE_INTERRUPT
            & ADCIntStatus(ADC_BASE, ADC_MIC_SEQ_NUM, true))
    {
    }

    uint32_t buffer;

    int32_t samplesRead = ADCSequenceDataGet(ADC_BASE, ADC_MIC_SEQ_NUM,
                                             &buffer);
    assert(samplesRead == 1 || samplesRead == 0);

    return buffer;
}

typedef struct
{
    uint32_t x;
    uint32_t y;
    uint32_t z;
} AccelReading;

// Reads the current value of the accelerometer.
AccelReading readAccel()
{
    enum
    {
        ADC_READ_VALUE_INTERRUPT = ADC_INT_SS1, // SS1 = sample sequence 1, can read up to 4 samples.
    };
    uint32_t buffer[4] = { 0 };
    int32_t samplesRead;
    do
    {
        ADCProcessorTrigger(ADC_BASE, ADC_ACCEL_SEQ_NUM);

        // Wait until the interrupt sampling the accelerometer value has completed.
        while (ADC_READ_VALUE_INTERRUPT
                & ADCIntStatus(ADC_BASE, ADC_ACCEL_SEQ_NUM, true))
        {
        }

        samplesRead = ADCSequenceDataGet(ADC_BASE, ADC_ACCEL_SEQ_NUM, buffer);
        assert(samplesRead >= 0 && samplesRead <= 4);
    }
    while (samplesRead != 3);
    // The above do-while loop technically discards legitimate readings, but doesn't require us to implement a queue.
    // It also assumes that whenever we get three samples, those samples are not offset with regards to which one is x, y, and z.

    AccelReading ret = { .x = buffer[0], .y = buffer[1], .z = buffer[2] };
    return ret;
}

int main(void)
{
    ConfigureUART();
    ButtonsInit();

    // Set the system clock to the same as in FreeRTOSConfig.h.
    SysCtlClockFreqSet(
    SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480,
                       SYSTEM_CLOCK);

    ADCSetup();

    while (true)
    {
        clearScreenAndMoveCursorHome();

        JoystickReading reading = readJoystick();
        char numberString[64] = { 0 };
        sprintf(numberString, "joy: hor(%d) ver(%d)\r\n", reading.horizontal,
                reading.vertical);
        uartPuts(numberString);
        while (UARTBusy(UART0_BASE))
        {

        }
        SysCtlDelay(SysCtlClockGet() / 2);
        AccelReading accelReading = readAccel();
        sprintf(numberString, "accel: x(%d) y(%d) z(%d)\r\n", accelReading.x,
                accelReading.y, accelReading.z);
        uartPuts(numberString);
        while (UARTBusy(UART0_BASE))
        {

        }
        SysCtlDelay(SysCtlClockGet() / 2);
        uint32_t micReading = readMicrophone();
        sprintf(numberString, "mic: %d\r\n", micReading);
        uartPuts(numberString);
        while (UARTBusy(UART0_BASE))
        {

        }

        SysCtlDelay(SysCtlClockGet() / 2);

    }
}

