#include "uartdriver.h"

// NOTE: According to the spec, these are the only includes that should be used.
#include <inc/tm4c129encpdt.h>
#include <stdint.h>

// FIXME: These includes are used for convenience during development, but should maybe be removed for the final version?
#include <assert.h>
#include <stdbool.h>

void UART_init(uint32_t base)
{
    // The driver should operate at 9600 baud.
    // The packet length should be 8 bits.
    // The driver should send no parity bit.
    // The driver should send one stop bit.
    // The driver should operate in normal channel mode.

    // Steps to do in initialization:
    // 1. Reset the driver to avoid unpredictable behavior during initialization.
    // 2. Set the baud rate.
    // 3. Set the message length.
    // 4. Initialize the stop bit.
    // 5. Set the driver to normal mode.
    // 6. Enable the communication.
    // 7. Enable the digital register bit for the GPIO pins.
    //
    // Check specification for more details.

    assert(false); // Not implemented.
}

char UART_getChar(void)
{
    // Make sure the driver has been initialized before accessing the hardware.

    assert(false); // Not implemented.
}

void UART_putChar(char c)
{
    // Make sure the driver has been initialized before accessing the hardware.

    assert(false); // Not implemented.
}

void UART_reset(void)
{
    assert(false); // Not implemented.
}

// FIXME: The lab spec doesn't show the arguments for this function, but this seems like the only logical one?
void UART_putString(const char *string)
{
    // TODO: According to the spec, this function should use UART_putChar.
    assert(false); // Not implemented.
}

// FIXME: The lab spec doesn't show the arguments for this function. I assume this is what they want?
void UART_getString(char *buf, uint32_t bufSize)
{
    // TODO: According to the spec, this function should use UART_getChar.
    assert(false); // Not implemented.
}
