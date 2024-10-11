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
int32_t
UARTCharGetInt(uint32_t ui32Base)
{
    //
    // Check the arguments.
    //
    ASSERT(_UARTBaseValid(ui32Base));

    //
    // Wait until a char is available.
    //
    while(HWREG(ui32Base + UART_O_FR) & UART_FR_RXFE)
    {
    }

    //
    // Now get the char.
    //
    return(HWREG(ui32Base + UART_O_DR));
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
    IntRegister(INT_UART0, UARTCharGetInt);

    // Enable the UART interrupt.
    MAP_IntEnable(INT_UART0);
    MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    UARTprintf("\033[2JEnter text: ");

    while (1)
    {
    }
}
