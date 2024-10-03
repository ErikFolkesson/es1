#include "timer.h"

#include "uartio.h"

#include <stdbool.h>
#include <assert.h>

#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"
#include "driverlib/uart.h"
#include "inc/tm4c129encpdt.h"
#include "inc/hw_memmap.h"
#include "inc/tm4c129encpdt.h"

// The value that g_counter gets reset to when the reset command is sent.
uint32_t g_counter_reset_value = 0;
// The counter representing the current time point of the timer in seconds.
uint32_t g_counter = 0;

// Constants.
enum
{
    TIMER_PERIPHERAL = SYSCTL_PERIPH_TIMER0, TIMER_BASE = TIMER0_BASE,
};

// The handler called whenever the timer counts down to 0.
// We use it to increment a counter representing the timer's current time point in seconds,
// and then print the clock representation of that counter using UART.
void timerHandler(void)
{
    // Get the interrupt status.
    uint32_t status = TimerIntStatus(TIMER_BASE, true);

    // Clear the asserted interrupts.
    TimerIntClear(TIMER_BASE, status);

    // If we reach 99:59:59, we wrap the timer back to 0.
    const uint32_t maxValue = 99 * 3600 + 59 * 60 + 60;

    g_counter++;
    g_counter %= maxValue;

    printClock(g_counter);
}

// Handles required setup of timer 0.
void setupTimer(void)
{
    uint32_t systemClock = SysCtlClockFreqSet(
            (SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL
                    | SYSCTL_CFG_VCO_480),
            16000);

    SysCtlPeripheralEnable(TIMER_PERIPHERAL);
    uint32_t timerConfig = TIMER_CFG_PERIODIC;
    TimerConfigure(TIMER_BASE, timerConfig);
    TimerLoadSet(TIMER_BASE, TIMER_A, systemClock);

    // Enable processor interrupts.
    MAP_IntMasterEnable();

    IntEnable(INT_TIMER0A);
    TimerIntRegister(TIMER_BASE, TIMER_A, timerHandler);
    TimerIntEnable(TIMER_BASE, TIMER_TIMA_TIMEOUT);

}

void startTimer(void)
{
    TimerEnable(TIMER_BASE, TIMER_A);
    printClock(g_counter);
}

void stopTimer(void)
{
    TimerDisable(TIMER_BASE, TIMER_A);
}

// Resets the timer to the user set or otherwise default initial value.
// The clock will also be printed at the same time.
void resetTimer(void)
{
    g_counter = g_counter_reset_value;
    printClock(g_counter);
}

// Sets the timer's current time point to the provided time. This will also become the new time point that gets used when resetting the timer.
// Preconditions: Seconds and minutes cannot be larger than 59, and hours cannot be larger than 99.
void setTimer(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    assert(hours < 100);
    assert(minutes < 60);
    assert(seconds < 60);

    g_counter_reset_value = hours * 3600 + minutes * 60 + seconds;
    resetTimer();
}
