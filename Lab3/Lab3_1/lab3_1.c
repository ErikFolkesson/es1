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

#include "driverlib/adc.h"

float pwm_word;
uint32_t systemClock;

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
            arrSize = 20
        };
        char buf[arrSize];
        UARTgets(buf, arrSize);
        UARTprintf("%s\n", buf);
    }
}

void UARTIntHandler2(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = MAP_UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    //
    MAP_UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while (MAP_UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        MAP_UARTCharPutNonBlocking(UART0_BASE,
                                   MAP_UARTCharGetNonBlocking(UART0_BASE));

        // MAP_UARTCharGetNonBlocking(UART0_BASE);
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

    UARTprintf("\033[2JEnter text: ");

    while (1)
    {
    }
}
