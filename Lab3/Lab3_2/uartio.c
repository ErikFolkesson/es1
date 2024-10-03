#include "uartio.h"

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <ctype.h>

#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "driverlib/gpio.h"
#include "inc/tm4c129encpdt.h"
#include "inc/hw_memmap.h"

extern void UARTIntHandler(void); // Declare the handler in lab3_2.c.

// We keep a buffer to store the text the user writes in the terminal.
#define USER_INPUT_BUF_CAPACITY 100
char userInputBuf[USER_INPUT_BUF_CAPACITY];
uint16_t userInputBufSize = 0;

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
    UARTStdioConfig(0, 115200, 16000000); // FIXME: Can remove if we don't use UARTprintf.

    // Enable processor interrupts.
    MAP_IntMasterEnable();

    // Register the interrupt handler function for UART 0.
    IntRegister(INT_UART0, UARTIntHandler);

    // Enable the UART interrupt.
    MAP_IntEnable(INT_UART0);
    MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
}


void uartPutChar(int c)
{
    UARTCharPut(UART0_BASE, c);
}


void uartPuts(const char *string)
{
    uint32_t i;
    size_t size = strlen(string);
    for (i = 0; i < size; i++)
    {
        UARTCharPut(UART0_BASE, string[i]);
    }
}


int32_t uartGetChar(void)
{
    int32_t c = UARTCharGet(UART0_BASE);
    if (isprint(c))
    {
        assert(userInputBufSize < USER_INPUT_BUF_CAPACITY - 1); // Avoid buffer overrun while keeping the null character in the string.
        userInputBuf[userInputBufSize++] = c;
        userInputBuf[userInputBufSize] = '\0';
        uartPutChar(c);
    }
    return c;
}


void uartPrintInt(int d) {
    int tenDigit = d / 10;
    // FIXME: This limits the int to 100 characters.
    assert(tenDigit < 10);
    int oneDigit = d % 10;
    uartPutChar('0' + tenDigit);
    uartPutChar('0' + oneDigit);
}

const char* uartGetInputBuf(void)
{
    return userInputBuf;
}


// Prints a clock in the format HH:MM:SS in position 1,1 in the terminal.
// FIXME: Should we take in hours, minutes, seconds instead of just seconds here?
void printClock(uint32_t counter)
{
    uint32_t hours = counter / 60 / 60;
    uint32_t minutes = counter / 60 % 60;
    uint32_t seconds = counter % 60;
    // FIXME: Use own functions instead of UARTprintf?
    moveCursorHome();
    uartPrintInt(hours);
    uartPutChar(':');
    uartPrintInt(minutes);
        uartPutChar(':');
        uartPrintInt(seconds);
        moveCursorToInputPos();

    //UARTprintf("\x1B[H%02u:%02u:%02u\x1B[3;%dH", hours, minutes, seconds,
    //           userInputBufSize + 1);
}

void moveCursorHome(void)
{
    uartPuts("\x1B[H");
}

void eraseLineAndReturnCarriage(void)
{
    // Erase entire line
    uartPuts("\x1B[2K");

    // Move cursor to the first column (ANSI columns are 1-based).
    uartPuts("\x1B[1G");

    userInputBufSize = 0;
    userInputBuf[0] = '\0';
}

void moveCursorToInputPos(void)
{
    uartPuts("\x1B[3;");
    int cursorColumn = userInputBufSize + 1;
    uartPrintInt(cursorColumn);
    uartPutChar('H');
}

void uartEraseChar(void)
{
    if (userInputBufSize > 0)
    {
        userInputBufSize--;
        userInputBuf[userInputBufSize] = '\0';
        uartPutChar('\b');
        // Sending a backspace to the terminal only moves the cursor, it doesn't remove any character.
        // To erase the character we print a space and then send another backspace.
        uartPutChar(' ');
        uartPutChar('\b');
    }
}