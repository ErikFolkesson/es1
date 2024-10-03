#include "uartio.h"
#include "timer.h"

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <assert.h>
#include <string.h>
#include <ctype.h>

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/timer.h"
#include "inc/hw_memmap.h"
#include "inc/tm4c129encpdt.h"

#define ARRSIZE(arr) (sizeof(arr) / sizeof(*arr))

uint8_t charToInt(char c)
{
    assert(isdigit(c));
    return c - '0';
}

bool isStartCommand(const char *input)
{
    return strcmp("start", input) == 0;
}

bool isResetCommand(const char *input)
{
    return strcmp("reset", input) == 0;
}

bool isStopCommand(const char *input)
{
    return strcmp("stop", input) == 0;
}

bool isClockCommand(const char *input)
{
    size_t size = strlen(input);
    if (size != 8)
    {
        return false;
    }

    bool colonsAreCorrect = input[2] == ':' && input[5] == ':';
    if (!colonsAreCorrect)
    {
        return false;
    }

    int numberPositions[] = { 0, 1, 3, 4, 6, 7 };
    int i;
    for (i = 0; i < ARRSIZE(numberPositions); i++)
    {
        if (!isdigit(input[numberPositions[i]]))
        {
            return false;
        }
    }

    return true;
}

//*****************************************************************************
//
// The UART interrupt handler.
//
//*****************************************************************************
void UARTIntHandler(void)
{
    // FIXME: Maybe move functionality to uart.c function instead.
    uint32_t ui32Status;

    // Get the interrupt status.
    ui32Status = MAP_UARTIntStatus(UART0_BASE, true);

    // Clear the asserted interrupts.
    MAP_UARTIntClear(UART0_BASE, ui32Status);

    enum
    {
        ENTER = '\r'
    };

    while (MAP_UARTCharsAvail(UART0_BASE))
    {
        // The only situation we have to take care of is when the user has finished inputting a command by pressing enter.
        int32_t c = uartGetChar();
        if (c == ENTER)
        {
            const char *userInputBuf = uartGetInputBuf();
            if (isStartCommand(userInputBuf))
            {
                startTimer();
            }
            else if (isStopCommand(userInputBuf))
            {
                stopTimer();
            }
            else if (isResetCommand(userInputBuf))
            {
                resetTimer();
            }
            else if (isClockCommand(userInputBuf))
            {
                uint8_t hours = charToInt(userInputBuf[0]) * 10
                        + charToInt(userInputBuf[1]);
                uint8_t minutes = charToInt(userInputBuf[3]) * 10
                        + charToInt(userInputBuf[4]);
                uint8_t seconds = charToInt(userInputBuf[6]) * 10
                        + charToInt(userInputBuf[7]);

                setTimer(hours, minutes, seconds);
            }

            eraseLineAndReturnCarriage();
        }
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

    setupTimer();

    uartPuts("\x1B[2J\x1B[2;1HEnter command below:\x1B[1E");

    while (1)
    {
    }
}
