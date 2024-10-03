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
        ENTER = '\r', ESC = 0x1B,
    };

    while (MAP_UARTCharsAvail(UART0_BASE))
    {
        int32_t c = uartGetChar();
        if (c == ENTER)
        {
            const char* userInputBuf = uartGetInputBuf();
            if (strcmp("start", userInputBuf) == 0)
            {
                startTimer();
            }
            else if (strcmp("stop", userInputBuf) == 0)
            {
                stopTimer();
            }
            else if (strcmp("reset", userInputBuf) == 0)
            {
                resetTimer();
            }
            else
            {
                size_t size = strlen(userInputBuf);
                if (size == 8)
                {
                    bool colonsAreCorrect = userInputBuf[2] == ':'
                            && userInputBuf[5] == ':';

                    int numberPositions[] = { 0, 1, 3, 4, 6, 7 };
                    bool numbersAreCorrect = true;
                    int i;
                    for (i = 0; i < ARRSIZE(numberPositions); i++)
                    {
                        if (!isdigit(userInputBuf[numberPositions[i]]))
                        {
                            numbersAreCorrect = false;
                            break;
                        }
                    }

                    if (colonsAreCorrect && numbersAreCorrect)
                    {
                        uint8_t hours = charToInt(userInputBuf[0]) * 10
                                + charToInt(userInputBuf[1]);
                        uint8_t minutes = charToInt(userInputBuf[3]) * 10
                                + charToInt(userInputBuf[4]);
                        uint8_t seconds = charToInt(userInputBuf[6]) * 10
                                + charToInt(userInputBuf[7]);

                        setTimer(hours, minutes, seconds);
                    }
                }
            }

            eraseLineAndReturnCarriage();


        }
        else if (c == '\b')
        {
            uartEraseChar();
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
