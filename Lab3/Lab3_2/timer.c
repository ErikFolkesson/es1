#include "timer.h"

#include "uartio.h"

#include <stdbool.h>
#include <assert.h>

#include <driverlib/timer.h>
#include "inc/hw_memmap.h"
#include "inc/tm4c129encpdt.h"
#include "driverlib/sysctl.h"


#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

#include "driverlib/interrupt.h"
#include "driverlib/rom_map.h"

#include "inc/tm4c129encpdt.h"
#include "driverlib/uart.h"




// FIXME: comments.
uint32_t g_counter_reset_value = 0;
uint32_t g_counter = 0;

enum
{
    TIMER_PERIPHERAL = SYSCTL_PERIPH_TIMER0, TIMER_BASE = TIMER0_BASE,
};

void timerHandler(void)
{
    // Get the interrupt status.
    uint32_t status = TimerIntStatus(TIMER_BASE, true);

    // Clear the asserted interrupts.
    TimerIntClear(TIMER_BASE, status);

    // TODO: Handle 99:59:59 wrapping??
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

// preconditions: seconds and minutes cannot be larger than 59. If hours is larger
void setTimer(uint8_t hours, uint8_t minutes, uint8_t seconds)
{
    assert(hours < 100);
    assert(minutes < 60);
    assert(seconds < 60);

    g_counter_reset_value = hours * 3600 + minutes * 60 + seconds;
    resetTimer();
}
