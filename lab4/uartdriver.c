#include "uartdriver.h"

// NOTE: According to the spec, these are the only includes that should be used.
#include <inc/tm4c129encpdt.h>
#include <stdint.h>

// FIXME: These includes are used for convenience during development, but should maybe be removed for the final version?
#include <assert.h>
#include <stdbool.h>

// FIXME: These are taken from inc/tm4c129encpdt.h. They are only here to collect the macros we might need in one place. They can be removed once we're done.
#define UART0_DR_R              (*((volatile uint32_t *)0x4000C000))
#define UART0_RSR_R             (*((volatile uint32_t *)0x4000C004))
#define UART0_ECR_R             (*((volatile uint32_t *)0x4000C004))
#define UART0_FR_R              (*((volatile uint32_t *)0x4000C018))
#define UART0_ILPR_R            (*((volatile uint32_t *)0x4000C020))
#define UART0_IBRD_R            (*((volatile uint32_t *)0x4000C024))
#define UART0_FBRD_R            (*((volatile uint32_t *)0x4000C028))
#define UART0_LCRH_R            (*((volatile uint32_t *)0x4000C02C))
#define UART0_CTL_R             (*((volatile uint32_t *)0x4000C030))
#define UART0_IFLS_R            (*((volatile uint32_t *)0x4000C034))
#define UART0_IM_R              (*((volatile uint32_t *)0x4000C038))
#define UART0_RIS_R             (*((volatile uint32_t *)0x4000C03C))
#define UART0_MIS_R             (*((volatile uint32_t *)0x4000C040))
#define UART0_ICR_R             (*((volatile uint32_t *)0x4000C044))
#define UART0_DMACTL_R          (*((volatile uint32_t *)0x4000C048))
#define UART0_9BITADDR_R        (*((volatile uint32_t *)0x4000C0A4))
#define UART0_9BITAMASK_R       (*((volatile uint32_t *)0x4000C0A8))
#define UART0_PP_R              (*((volatile uint32_t *)0x4000CFC0))
#define UART0_CC_R              (*((volatile uint32_t *)0x4000CFC8))

#define GPIO_PORTA_AHB_DATA_BITS_R                                             \
                                ((volatile uint32_t *)0x40058000) // FIXME: This doesn't dereference?? I assume we probably don't need to read from it since we use the UART data register instead, so probably fine?
#define GPIO_PORTA_AHB_DATA_R   (*((volatile uint32_t *)0x400583FC))
#define GPIO_PORTA_AHB_DIR_R    (*((volatile uint32_t *)0x40058400))
#define GPIO_PORTA_AHB_IS_R     (*((volatile uint32_t *)0x40058404))
#define GPIO_PORTA_AHB_IBE_R    (*((volatile uint32_t *)0x40058408))
#define GPIO_PORTA_AHB_IEV_R    (*((volatile uint32_t *)0x4005840C))
#define GPIO_PORTA_AHB_IM_R     (*((volatile uint32_t *)0x40058410))
#define GPIO_PORTA_AHB_RIS_R    (*((volatile uint32_t *)0x40058414))
#define GPIO_PORTA_AHB_MIS_R    (*((volatile uint32_t *)0x40058418))
#define GPIO_PORTA_AHB_ICR_R    (*((volatile uint32_t *)0x4005841C))
#define GPIO_PORTA_AHB_AFSEL_R  (*((volatile uint32_t *)0x40058420))
#define GPIO_PORTA_AHB_DR2R_R   (*((volatile uint32_t *)0x40058500))
#define GPIO_PORTA_AHB_DR4R_R   (*((volatile uint32_t *)0x40058504))
#define GPIO_PORTA_AHB_DR8R_R   (*((volatile uint32_t *)0x40058508))
#define GPIO_PORTA_AHB_ODR_R    (*((volatile uint32_t *)0x4005850C))
#define GPIO_PORTA_AHB_PUR_R    (*((volatile uint32_t *)0x40058510))
#define GPIO_PORTA_AHB_PDR_R    (*((volatile uint32_t *)0x40058514))
#define GPIO_PORTA_AHB_SLR_R    (*((volatile uint32_t *)0x40058518))
#define GPIO_PORTA_AHB_DEN_R    (*((volatile uint32_t *)0x4005851C))
#define GPIO_PORTA_AHB_LOCK_R   (*((volatile uint32_t *)0x40058520))
#define GPIO_PORTA_AHB_CR_R     (*((volatile uint32_t *)0x40058524))
#define GPIO_PORTA_AHB_AMSEL_R  (*((volatile uint32_t *)0x40058528))
#define GPIO_PORTA_AHB_PCTL_R   (*((volatile uint32_t *)0x4005852C))
#define GPIO_PORTA_AHB_ADCCTL_R (*((volatile uint32_t *)0x40058530))
#define GPIO_PORTA_AHB_DMACTL_R (*((volatile uint32_t *)0x40058534))
#define GPIO_PORTA_AHB_SI_R     (*((volatile uint32_t *)0x40058538))
#define GPIO_PORTA_AHB_DR12R_R  (*((volatile uint32_t *)0x4005853C))
#define GPIO_PORTA_AHB_WAKEPEN_R                                              \
                                (*((volatile uint32_t *)0x40058540))
#define GPIO_PORTA_AHB_WAKELVL_R                                              \
                                (*((volatile uint32_t *)0x40058544))
#define GPIO_PORTA_AHB_WAKESTAT_R                                             \
                                (*((volatile uint32_t *)0x40058548))
#define GPIO_PORTA_AHB_PP_R     (*((volatile uint32_t *)0x40058FC0))
#define GPIO_PORTA_AHB_PC_R     (*((volatile uint32_t *)0x40058FC4))

#define RCGCUART_REG SYSCTL_RCGCUART_R
#define RCGCGPIO_REG SYSCTL_RCGCGPIO_R

#define PIOSC_VALUE UART_CC_CS_PIOSC

// Line control bits.
#define LINE_CONTROL_FEN UART_LCRH_FEN // Enable/disable FIFO
#define LINE_CONTROL_MSG_LEN UART_LCRH_WLEN_8

// RCGCUART bits.
//SYSCTL_RCGCUART_R0

// CTL bits.
#define CTL_UARTEN UART_CTL_UARTEN
#define CTL_UART_TX_EN UART_CTL_TXE
#define CTL_UART_RX_EN UART_CTL_RXE

// FLAG bits.
#define FLAG_BUSY UART_FR_BUSY
#define FLAG_RXFE UART_FR_RXFE // Receive FIFO empty
#define FLAG_TXFF UART_FR_TXFF // Transmit FIFO full

// FIXME: Figure out if this is allowed/if we want to do it another way.
#define UART0_BASE              0x4000C000

#define REG(n) (*((volatile uint32_t*) (n)))

#define GPIO_PORT_A_BASE 0x40058000U

#define GPIO_PIN_0 (1U << 0U)
#define GPIO_PIN_1 (1U << 1U)

#define BAUDCLOCK 16000000U
#define BAUDRATE 9600U
#define CLKDIV 8U // FIXME: might be 16
// FIXME: They do the calculation differently in UARTConfigSetExpClk
#define BRD ((double) BAUDCLOCK / (CLKDIV * BAUDRATE))
#define BRDINT ((uint32_t) BRD)
#define BRDFRAC ((uint32_t) ((BRD - BRDINT) * 64 + 0.5))

typedef struct
{
    uint32_t value;
} UartBase;

typedef struct
{
    uint32_t value;
} GpioBase;

static UartBase g_uartBase;
static GpioBase g_gpioBase;

// FIXME: If we are supposed to only support UART0, why do we receive uartBase here??
void UART_init(uint32_t uartBase)
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
    g_uartBase.value = UART0_BASE;
    g_gpioBase.value = GPIO_PORT_A_BASE;

    //     1.1 Enable UART module using RCGCUART register (page 395)
    RCGCUART_REG = SYSCTL_RCGCUART_R0;
    //     1.2 Enable clock to appropriate GPIO module via RCGCGPIO register (page 389). GPIO port enabling info in table 29-5 (page 1932).

    //         Set clock source
    UART0_CC_R = PIOSC_VALUE;
    RCGCGPIO_REG = SYSCTL_RCGCGPIO_R0;
    //     1.3 Set GPIO AFSEL bits for appropriate pins (page 778). To determine GPIOs to configure, see table 29-4 (page 1921)
    GPIO_PORTA_AHB_AFSEL_R |= GPIO_PIN_0 | GPIO_PIN_1;
    //     1.4 Configure GPIO current level and/or slew rate as specified for mode selected (page 780 and 788).
    // FIXME: Set to 2mA (GPIODR2R), offset 0x500 (Is already set by default)
    // Set slew rate: FIXME: (According to data sheet 2mA doesn't need to?)

    //     1.5 Configure PMCn fields in GPIOPCTL register to assign UART signals to appropriate pins (page 795, table 29-5 page 1932).
    // FIXME: Need to write a 1 to all parts of the register that correspond to the pins we use. Can probably use loop over bits and do 1 << (i * 4). For now we hard code.
    GPIO_PORTA_AHB_PCTL_R |= 17U; // 4th and 0th bit set.

    // What is done in SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA):

    // What is done in SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0).
    // What is done in GPIOPinConfigure(GPIO_PA0_U0RX).
    // What is done in GPIOPinConfigure(GPIO_PA1_U0TX).
    // What is done in GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1):
    //   GPIODirModeSet(port, pins, GPIO_DIR_MODE_HW)
    //     Sets the pin direction as input (since UART takes care of it?) (it should be defaulted to input. FIXME: Should we still set it to input manually? Probably, in case the user used the pin previously?)
    //     Sets the pin modes to be their alternate functions (we've already taken care of it).
    //   GPIOPadConfigSet(port, pins, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD)
    //     Sets the GPIO peripheral configuration register
    //     Sets the output drive strength to 2mA and sets slew rate (writes to DR2R, DR4R, DR8R, and SLR registers)
    //     Sets the DR12R register FIXME: This register doesn't show in the data sheet.
    //     Sets the pin type (registers ODR, PUR, PDR, DEN)
    //     Sets the WAKELVL and WAKEPEN registers.
    //     Sets the AMSEL register.

    // 2. Set the baud rate.
    // FIXME: "This internal register [UARTLCRH] is only updated when a write operation to UARTLCRH is performed, so any changes to the baud-rate divisor must be followed by a write to the UARTLCRH register for the changes to take effect."
    // FIXME: We try disabling UART before writing this????
    UART0_FBRD_R = BRDFRAC;
    UART0_IBRD_R = BRDINT;
    // 3. Set the message length.
    // Parity bit set to 0 disables and stop bit set to 0 uses one stop bit, so those are implicitly the correct values.
    UART0_LCRH_R |= LINE_CONTROL_MSG_LEN | UART_LCRH_FEN;
    // 5. Set the driver to normal mode.
    // FIXME: It seems like it might be normal mode by default?

    // 6. Enable the communication.
    // Enable UART and tx/rx.
    // FIXME: Break error and framing error occurs due to this.
    UART0_CTL_R |= CTL_UARTEN | CTL_UART_TX_EN | CTL_UART_RX_EN;

    // 7. Enable the digital register bit for the GPIO pins.
    // FIXME: Do we need to do something to open drain, pull up, and pull down?
    // FIXME: Is this done in steps 1.2/1.3?
    // FIXME: Don't hard code if we need to handle multiple ports.
    GPIO_PORTA_AHB_DEN_R = 0x3;

    // Check specification for more details.
}

char UART_getChar(void)
{
    // Make sure the driver has been initialized before accessing the hardware.
    // Do this by checking that we have a non-zero base
    assert(g_uartBase.value != 0);

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
    assert(g_uartBase.value != 0);

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
    g_uartBase.value = 0;
    g_gpioBase.value = 0;
}

// FIXME: The lab spec doesn't show the arguments for this function, but this seems like the only logical one?
void UART_putString(const char *string)
{
    // Get the length of the string.
    uint16_t len = sizeof(string) / sizeof(string[0]);
    uint16_t i;

    // Loop through the string, sending each char to UART_putChar()
    for (i = 0; i < len; i++) {
        UART_putChar(string[i]);
    }
}

// FIXME: The lab spec doesn't show the arguments for this function. I assume this is what they want?
void UART_getString(char *buf, uint32_t bufSize)
{
    uint16_t i;

    // Keep reading characters until the entire buffer is full
    // or until we get a newline character.
    for (i = 0; i < bufSize; i++) {
        char c = UART_getChar();
        if (c == '\n') {
            // We have got a newline character, terminate
            break;
        }
        buf[i] = c;
    }
}
