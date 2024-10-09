#include "uartdriver.h"

// NOTE: According to the spec, these are the only includes that should be used.
#include <inc/tm4c129encpdt.h>
#include <stdint.h>

// FIXME: These includes are used for convenience during development, but should maybe be removed for the final version?
#include <assert.h>
#include <stdbool.h>

// Register offsets
#define CC_REG_OFFSET 0xFC8U
#define CTL_REG_OFFSET 0x030U
#define LINE_CONTROL_REG_OFFSET 0x02CU

#define RCGCUART_REG SYSCTL_RCGCUART_R

// Line control bits.
#define LINE_CONTROL_FEN 0x8U // Enable/disable FIFO

// RCGCUART bits.
SYSCTL_RCGCUART_R0

// CTL bits.
#define CTL_UARTEN UART_CTL_UARTEN

// FLAG bits.
#define FLAG_BUSY 0x08U

// FIXME: Figure out if this is allowed/if we want to do it another way.
#define UART0_BASE              0x4000C000
#define UART1_BASE              0x4000D000
#define UART2_BASE              0x4000E000
#define UART3_BASE              0x4000F000
#define UART4_BASE              0x40010000
#define UART5_BASE              0x40011000
#define UART6_BASE              0x40012000
#define UART7_BASE              0x40013000

#define REG(n) (*((volatile uint32_t*) (n)))

uint32_t g_base;

uint32_t to_rcgcuart_bit(uint32_t uartBase) {
    switch (uartBase) {
    case UART0_BASE: return SYSCTL_RCGCUART_R0;
    case UART1_BASE: return SYSCTL_RCGCUART_R1;
    case UART2_BASE: return SYSCTL_RCGCUART_R2;
    case UART3_BASE: return SYSCTL_RCGCUART_R3;
    case UART4_BASE: return SYSCTL_RCGCUART_R4;
    case UART5_BASE: return SYSCTL_RCGCUART_R5;
    case UART6_BASE: return SYSCTL_RCGCUART_R6;
    case UART7_BASE: return SYSCTL_RCGCUART_R7;
    }
    assert(0); // Faulty uart base provided.
}

void UART_init(uint32_t base)
{
    // The driver should operate at 9600 baud.
    // The packet length should be 8 bits.
    // The driver should send no parity bit.
    // The driver should send one stop bit.
    // The driver should operate in normal channel mode.

    // Steps to do in initialization:
    // 1. Reset the driver to avoid unpredictable behavior during initialization.
    // FIXME: Do we need to reset or not?
    // UART_reset();
    g_base = base;
    //     1.1 Enable UART module using RCGCUART register (page 395)

    //     1.2 Enable clock to appropriate GPIO module via RCGCGPIO register (page 389). GPIO port enabling info in table 29-5 (page 1932).

    //     1.3 Set GPIO AFSEL bits for appropriate pins (page 778). To determine GPIOs to configure, see table 29-4 (page 1921)

    //     1.4 Configure GPIO current level and/or slew rate as specified for mode selected (page 780 and 788).

    //     1.5 Configure PMCn fields in GPIOPCTL register to assign UART signals to appropriate pins (page 795, table 29-5 page 1932).

    // 2. Set the baud rate.
    // FIXME: "This internal register [UARTLCRH] is only updated when a write operation to UARTLCRH is performed, so any changes to the baud-rate divisor must be followed by a write to the UARTLCRH register for the changes to take effect."

    // 3. Set the message length.

    // 4. Initialize the stop bit.

    // 5. Set the driver to normal mode.

    // 6. Enable the communication.

    // 7. Enable the digital register bit for the GPIO pins.

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
    // Write data using UARTDR register.

    assert(false); // Not implemented.
}

void UART_reset(void)
{
    // Reset both UART and GPIO modules

    // The UARTCTL register is the control register.
    // All the bits are cleared on reset except for the Transmit Enable (TXE) and Receive Enable (RXE) bits, which are set.
    // 1. Disable the UART.
    // FIXME: Do we need to disable the tx and rx bits as well?
    REG(g_base + CTL_REG_OFFSET) &= ~CTL_UARTEN;
    // 2. Wait for the end of transmission or reception of the current character.
    // FIXME: This might not check for reception?
    //        Maybe fix: "When the receiver is idle (the UnRx signal is continuously 1)"
    while ((REG(g_base + FLAG_REG_OFFSET) & FLAG_BUSY) != 0)
    {
    }
    // 3. Flush the transmit FIFO by clearing bit 4 (FEN) in the line control register (UARTLCRH).
    REG(g_base + LINE_CONTROL_REG_OFFSET) &= ~LINE_CONTROL_FEN;
    // FIXME: Maybe we need to wait a bit before re-enabling? Maybe we don't need to re-enable at all?
    REG(g_base + LINE_CONTROL_REG_OFFSET) |= LINE_CONTROL_FEN;

    // 4. Reset interrupt bits.
    REG(g_base + INT_CLEAR_REG_OFFSET) = ~0U;

    // 5. Enable the UART
    REG(g_base + CTL_REG_OFFSET) |= CTL_UARTEN;
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
