#include "uartdriver.h"

// NOTE: According to the spec, these are the only includes that should be used.
#include <inc/tm4c129encpdt.h>
#include <stdint.h>

// FIXME: These includes are used for convenience during development, but should maybe be removed for the final version?
#include <assert.h>
#include <stdbool.h>

#define RCGCUART_REG SYSCTL_RCGCUART_R
#define RCGCGPIO_REG SYSCTL_RCGCGPIO_R

#define PIOSC_VALUE UART_CC_CS_PIOSC

// Line control bits.
#define LINE_CONTROL_FEN UART_LCRH_FEN // Enable/disable FIFO
#define LINE_CONTROL_MSG_LEN UART_LCRH_WLEN_8

// CTL bits.
#define CTL_UARTEN UART_CTL_UARTEN
#define CTL_UART_TX_EN UART_CTL_TXE
#define CTL_UART_RX_EN UART_CTL_RXE

// FLAG bits.
#define FLAG_BUSY UART_FR_BUSY
#define FLAG_RXFE UART_FR_RXFE // Receive FIFO empty
#define FLAG_TXFF UART_FR_TXFF // Transmit FIFO full

#define GPIO_PIN_0 (1U << 0U)
#define GPIO_PIN_1 (1U << 1U)

#define BAUDCLOCK 16000000U
#define BAUDRATE 9600U
#define CLKDIV 16U
#define BRD ((double) BAUDCLOCK / (CLKDIV * BAUDRATE))
#define BRDINT ((uint32_t) BRD)
#define BRDFRAC ((uint32_t) ((BRD - BRDINT) * 64 + 0.5))

static bool g_initialized;

// FIXME: If we are supposed to only support UART0, why do we receive uartBase here??
void UART_init(void)
{
    // The driver should operate at 9600 baud.
    // The packet length should be 8 bits.
    // The driver should send no parity bit.
    // The driver should send one stop bit.
    // The driver should operate in normal channel mode.

    // Start from a clean state.
    // UART_reset();

    // Enable the UART module.
    RCGCUART_REG = SYSCTL_RCGCUART_R0; // "There must be a delay of 3 system clocks after the UART module clock is enabled before any UART module registers are accessed."

    // Enable the GPIO module clock.
    RCGCGPIO_REG = SYSCTL_RCGCGPIO_R0; // "There must be a delay of 3 system clocks after the GPIO module clock is enabled before any GPIO module registers are accessed."

    // Set the UART clock to be PIOSC.
    UART0_CC_R = PIOSC_VALUE;

    // Set GPIO pins 0 and 1 to be AFSEL (Alternate Function Select).
    GPIO_PORTA_AHB_AFSEL_R |= GPIO_PIN_0 | GPIO_PIN_1;

    // Assign the GPIO pins to UART functionality.
    GPIO_PORTA_AHB_PCTL_R |= 17U; // 4th and 0th bit set.

    // Set the baud rate.
    UART0_FBRD_R = BRDFRAC;
    UART0_IBRD_R = BRDINT;

    // Set the message length.
    // By default, the parity is disabled and one stop bit is used, so we don't explicitly set them.
    UART0_LCRH_R |= LINE_CONTROL_MSG_LEN;

    // Enable the digital register bit for the GPIO pins.
    GPIO_PORTA_AHB_DEN_R = 0x3;

    // Enable UART.
    UART0_CTL_R |= CTL_UARTEN;

    g_initialized = true;
}

char UART_getChar(void)
{
    // Make sure the driver has been initialized before accessing the hardware.
    // Do this by checking that we have a non-zero base
    assert(g_initialized);

    // Wait until the receive FIFO is not empty
    while (UART0_FR_R & FLAG_RXFE)
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
    while (UART0_FR_R & FLAG_TXFF)
    {
    }

    // Transmit FIFO is not full
    // Write data using UARTDR register.
    UART0_DR_R = c;
}

void UART_reset(void)
{
    // FIXME: Figure out what to actually do here.

    // Reset both UART and GPIO modules

    // The UARTCTL register is the control register.
    // All the bits are cleared on reset except for the Transmit Enable (TXE) and Receive Enable (RXE) bits, which are set.
    // 1. Disable the UART.
    // FIXME: Do we need to disable the tx and rx bits as well?
    UART0_CTL_R &= ~CTL_UARTEN;
    // 2. Wait for the end of transmission or reception of the current character.
    // FIXME: This might not check for reception?
    //        Maybe fix: "When the receiver is idle (the UnRx signal is continuously 1)"
    while ((UART0_FR_R & FLAG_BUSY) != 0)
    {
    }
    // 3. Flush the transmit FIFO by clearing bit 4 (FEN) in the line control register (UARTLCRH).
    UART0_LCRH_R &= ~LINE_CONTROL_FEN;
    // FIXME: Maybe we need to wait a bit before re-enabling? Maybe we don't need to re-enable at all?
    UART0_LCRH_R |= LINE_CONTROL_FEN;

    // 4. Reset interrupt bits.
    // FIXME: Should the gpio interrupts be cleared as well?
    GPIO_PORTA_AHB_ICR_R = ~0U;
    UART0_ICR_R = ~0U;

    // 5. Enable the UART
    UART0_CTL_R |= CTL_UARTEN;

    // FIXME: In spec, the program should error when trying to read string after resetting without initing.
    //        Seems like the UART should be reset?
    g_initialized = false;
}

// FIXME: The lab spec doesn't show the arguments for this function, but this seems like the only logical one?
void UART_putString(const char *string)
{
    // Loop through the string until we find the null character, sending each char to UART_putChar()
    for (const char *it = string; *it != '\0'; it++)
    {
        UART_putChar(*it);
    }
}

// FIXME: The lab spec doesn't show the arguments for this function. I assume this is what they want? Ideally this would also return the # of characters read.
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
