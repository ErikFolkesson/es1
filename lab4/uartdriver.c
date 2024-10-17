#include "uartdriver.h"

// Mandatory includes.
#include <inc/tm4c129encpdt.h>
#include <stdint.h>

// Convenience includes.
#include <assert.h>
#include <stdbool.h>

#define GPIO_PIN_0 (1U << 0U)
#define GPIO_PIN_1 (1U << 1U)

#define BAUDCLOCK 16000000U
#define BAUDRATE 9600U
// ClkDiv should be 16 if High Speed Enable is not set.
#define CLKDIV 16U

// Baud Rate Divisor calculation.
#define BRD ((double) BAUDCLOCK / (CLKDIV * BAUDRATE))
#define BRDINT ((uint32_t) BRD)
#define BRDFRAC ((uint32_t) ((BRD - BRDINT) * 64 + 0.5))

static bool g_initialized;

void UART_init(void)
{
    // The driver should operate at 9600 baud.
    // The packet length should be 8 bits.
    // The driver should send no parity bit.
    // The driver should send one stop bit.
    // The driver should operate in normal channel mode.

    // Start from a clean state.
    UART_reset();

    // Enable the UART module.
    SYSCTL_RCGCUART_R = SYSCTL_RCGCUART_R0; // "There must be a delay of 3 system clocks after the UART module clock is enabled before any UART module registers are accessed."

    // Enable the GPIO module clock.
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R0; // "There must be a delay of 3 system clocks after the GPIO module clock is enabled before any GPIO module registers are accessed."

    // Set the UART clock to be PIOSC.
    UART0_CC_R = UART_CC_CS_PIOSC;

    // Set GPIO pins 0 and 1 to be AFSEL (Alternate Function Select).
    GPIO_PORTA_AHB_AFSEL_R |= GPIO_PIN_0 | GPIO_PIN_1;

    // Assign the GPIO pins to UART functionality.
    GPIO_PORTA_AHB_PCTL_R |= 17U; // 4th and 0th bit set.

    // Set the baud rate.
    UART0_FBRD_R = BRDFRAC;
    UART0_IBRD_R = BRDINT;

    // Set the message length.
    // By default, the parity is disabled and one stop bit is used, so we don't explicitly set them.
    UART0_LCRH_R |= UART_LCRH_WLEN_8;

    // Enable the digital register bit for the GPIO pins.
    GPIO_PORTA_AHB_DEN_R = 0x3;

    // Enable UART.
    UART0_CTL_R |= UART_CTL_UARTEN;

    g_initialized = true;
}

char UART_getChar(void)
{
    // Make sure the driver has been initialized before accessing the hardware.
    // Do this by checking that we have a non-zero base
    assert(g_initialized);

    // Wait until the receive FIFO is not empty
    while (UART0_FR_R & UART_FR_RXFE)
    {
    }

    // FIFO is non-empty, read and return the next character.
    return UART0_DR_R;
}

void UART_putChar(char c)
{
    // Make sure the driver has been initialized before accessing the hardware.
    // Do this by checking that we have a non-zero base
    assert(g_initialized);

    // Wait until transmit FIFO no longer is full
    while (UART0_FR_R & UART_FR_TXFF)
    {
    }

    // Transmit FIFO is not full
    // Write data using UARTDR register.
    UART0_DR_R = c;
}

// Resets UART0. After a call to this function, UART_init must be called before reading or writing to UART.
void UART_reset(void)
{
    // We don't attempt the reset if we haven't yet initialized (Accessing some of the registers is illegal without initialization).
    if (!g_initialized) {
        return;
    }

    // Wait for current transmission to finish.
    while ((UART0_FR_R & UART_FR_BUSY) != 0)
    {
    }

    // Disable UART.
    UART0_CTL_R &= ~UART_CTL_UARTEN;

    // Disable digital enable for the GPIO pins.
    GPIO_PORTA_AHB_DEN_R = ~0x3;

    // Clear the message length.
    UART0_LCRH_R &= ~UART_LCRH_WLEN_8;

    // Clear the baud rate.
    UART0_FBRD_R = 0;
    UART0_IBRD_R = 0;

    // Unassign UART signals for the GPIO pins.
    GPIO_PORTA_AHB_PCTL_R &= ~17U; // 4th and 0th bit set.

    // Unset GPIO AFSEL for the pins.
    GPIO_PORTA_AHB_AFSEL_R &= ~(GPIO_PIN_0 | GPIO_PIN_1);

    // Clear interrupts.
    GPIO_PORTA_AHB_ICR_R = ~0U;
    UART0_ICR_R = ~0U;

    // Unset PIOSC as the UART clock.
    UART0_CC_R &= ~UART_CC_CS_PIOSC;

    // Disable the GPIO module.
    SYSCTL_RCGCGPIO_R &= ~SYSCTL_RCGCGPIO_R0;

    // Disable the UART module.
    SYSCTL_RCGCUART_R &= ~SYSCTL_RCGCUART_R0;

    g_initialized = false;
}

// Writes the provided NUL-terminated string to UART0.
void UART_putString(const char *string)
{
    // Loop through the string until we find the null character, sending each char to UART_putChar()
    for (const char *it = string; *it != '\0'; it++)
    {
        UART_putChar(*it);
    }
}

// Receives a string of characters into the provided buffer and terminates it with a NUL-character.
// The maximum amount of characters read is `bufSize - 1`.
void UART_getString(char *buf, uint32_t bufSize)
{
    uint16_t i;

    // Keep reading characters until the entire buffer is full
    // or until we get a newline character.
    for (i = 0; i < bufSize - 1; i++)
    {
        char c = UART_getChar();
        if (c == '\r')
        {
            // We have got a newline character, terminate
            break;
        }
        buf[i] = c;
    }
    buf[i] = '\0'; // Make sure we terminate the written string.
}
