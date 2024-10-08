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

    uint32_t baudRate = 115200;
    // 8 data bits, 1 stop bit, and no parity bit.
    uint32_t uartConfig = UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE
            | UART_CONFIG_PAR_NONE;
    // For PIOSC clock, the frequency should be set as 16'000'000
    uint32_t systemClockFrequency = 16000000;
    UARTConfigSetExpClk(UART0_BASE, systemClockFrequency, baudRate, uartConfig);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Enable processor interrupts.
    MAP_IntMasterEnable();

    // Register the interrupt handler function for UART 0.
    IntRegister(INT_UART0, UARTIntHandler);

    // Enable the UART interrupt.
    MAP_IntEnable(INT_UART0);
    MAP_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    UARTEnable(UART0_BASE);
}

// Prints a char using UART.
void uartPutChar(int c)
{
    UARTCharPut(UART0_BASE, c);
}

// Prints a string using UART. No newline is appended to the output.
void uartPuts(const char *string)
{
    uint32_t i;
    size_t size = strlen(string);
    for (i = 0; i < size; i++)
    {
        UARTCharPut(UART0_BASE, string[i]);
    }
}

// Reads and returns a char using UART.
// Also adds read printable characters to an input buffer, or erases a character from the buffer if the read character is a backspace.
// Read characters are also printed to the terminal to reflect the state of the input buffer.
// This operation will block until a character is available for reading.
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
    else if (c == '\b')
    {
        uartEraseChar();
    }
    return c;
}

// Prints the string representation of the provided int using UART.
// Precondition: The provided int cannot be larger than 99.
void uartPrintInt(int number)
{
    int tenDigit = number / 10;
    assert(tenDigit < 10); // Don't handle ints larger than 99.
    int oneDigit = number % 10;
    uartPutChar('0' + tenDigit);
    uartPutChar('0' + oneDigit);
}

// Returns a pointer to the input buffer containing the input that the user has typed into the terminal.
const char* uartGetInputBuf(void)
{
    return userInputBuf;
}

// Prints a clock in the format HH:MM:SS in position 1,1 in the terminal.
void printClock(uint32_t seconds)
{
    uint32_t hours = seconds / 60 / 60;
    uint32_t minutes = seconds / 60 % 60;
    uint32_t remainingSeconds = seconds % 60;

    moveCursorHome();
    uartPrintInt(hours);
    uartPutChar(':');
    uartPrintInt(minutes);
    uartPutChar(':');
    uartPrintInt(remainingSeconds);
    moveCursorToInputPos();
}

// Prints the ANSI escape sequence to move the cursor to the home position (1,1)
void moveCursorHome(void)
{
    uartPuts("\x1B[H");
}

// Prints the ANSI escape sequences to erase the current line and move the cursor to the first column.
// The input buffer is also cleared.
void eraseLineAndReturnCarriage(void)
{
    // Erase entire line
    uartPuts("\x1B[2K");

    // Move cursor to the first column (ANSI columns are 1-based).
    uartPuts("\x1B[1G");

    userInputBufSize = 0;
    userInputBuf[0] = '\0';
}

// Prints the ANSI sequence to move the cursor to the end of the current user input.
void moveCursorToInputPos(void)
{
    uartPuts("\x1B[3;");
    int cursorColumn = userInputBufSize + 1;
    uartPrintInt(cursorColumn);
    uartPutChar('H');
}

// Erases a character in both the terminal and the user input buffer.
void uartEraseChar(void)
{
    if (userInputBufSize > 0)
    {
        userInputBufSize--;
        userInputBuf[userInputBufSize] = '\0';
        uartPutChar('\b');
        // Sending a backspace to the terminal only moves the cursor, it doesn't remove any character.
        // To erase the character we need to also print a space and then send another backspace.
        uartPutChar(' ');
        uartPutChar('\b');
    }
}
