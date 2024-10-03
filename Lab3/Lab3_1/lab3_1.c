//*****************************************************************************
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <assert.h>
#include <ctype.h>

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/adc.h"
#include "inc/tm4c129encpdt.h"
#include "inc/hw_memmap.h"

float pwm_word;
uint32_t systemClock;

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

void moveCursorToStartOfNextLine(void)
{
    uartPuts("\x1B[1E");
}
void moveCursorToStartOfPreviousLine(void)
{
    uartPuts("\x1B[1F");
}

void eraseCurrentLine(void)
{
    uartPuts("\x1B[2K");
}

void eraseScreen(void)
{
    uartPuts("\x1B[2J");
}

void moveCursorHome(void)
{
    uartPuts("\x1B[H");
}

void moveCursorToStartOfLine(void)
{
    uartPuts("\x1B[1G");
}

int main(void)
{
    ConfigureUART();

    eraseScreen();
    moveCursorHome();

    enum
    {
        ENTER = '\r', BACKSPACE = '\b', USER_INPUT_BUF_CAPACITY = 100
    };
    char userInputBuf[USER_INPUT_BUF_CAPACITY] = { 0 };
    int userInputBufSize = 0;

    while (1)
    {
        if (UARTCharsAvail(UART0_BASE))
        {

            int c = UARTCharGet(UART0_BASE);
            if (c == ENTER)
            {
                moveCursorToStartOfNextLine();
                eraseCurrentLine();

                uartPuts(userInputBuf);
                moveCursorToStartOfPreviousLine();
                eraseCurrentLine();

                userInputBuf[0] = '\0';
                userInputBufSize = 0;
            }
            else if (isprint(c))
            {
                assert(userInputBufSize < USER_INPUT_BUF_CAPACITY - 1); // Avoid buffer overrun while keeping the null character in the string.
                userInputBuf[userInputBufSize++] = c;
                userInputBuf[userInputBufSize] = '\0';
                UARTCharPut(UART0_BASE, c);
            }
            else if (c == BACKSPACE)
            {
                if (userInputBufSize > 0)
                {
                    userInputBufSize--;
                    userInputBuf[userInputBufSize] = '\0';
                    // Sending a backspace to the terminal only moves the cursor, it doesn't remove any character.
                    // To erase the character we need to also print a space and then send another backspace.
                    uartPuts("\b \b");
                }
            }

        }

    }
}
