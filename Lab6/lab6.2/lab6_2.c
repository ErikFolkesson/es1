#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <ctype.h>
#include <stdio.h>

#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"

#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "inc/tm4c129encpdt.h"

SemaphoreHandle_t g_printMutex;

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

    UARTEnable(UART0_BASE);
}

void uartPuts(const char *str)
{
    const char *it;
    for (it = str; *it != '\0'; it++)
    {
        UARTCharPut(UART0_BASE, *it);
    }
}

enum {
    BUF_CAPACITY = 15
};

char buf[BUF_CAPACITY + 1];
int bufSize;
int totalCharacterCount;

// Prints the ANSI escape sequence to move the cursor to the home position (1,1)
void clearScreenAndMoveCursorHome(void)
{
    uartPuts("\x1B[2J");
    uartPuts("\x1B[H");
}

// If the call needs to be thread-safe, the caller is responsible for locking a mutex.
void printUartData(void) {
    clearScreenAndMoveCursorHome();
    uartPuts(buf);
    // TODO: poll status semaphore.
}

// Rotates te element in the buffer one step to the left, with the first character moving to the last position.
void rotateLeft(char*buf, int bufSize) {
    char tmp = *buf;
    for (int i = 1; i < bufSize; i++) {
        buf[i - 1] = buf[i];
    }
    buf[bufSize - 1] = tmp;
}

void uartTask(void* parameters) {
    while (true) {
        int32_t c = UARTCharGet(UART0_BASE);
        if (!isprint(c)) {
            continue;
        }
        xSemaphoreTake(g_printMutex, portMAX_DELAY);

        if (bufSize == BUF_CAPACITY) {
            rotateLeft(buf, bufSize);
            buf[bufSize - 1] = c;
        } else {
            buf[bufSize] = c;
            bufSize++;
        }
        printUartData();
        xSemaphoreGive(g_printMutex);
    }
}

int main(void)
{
    ConfigureUART();

    // Set the system clock to the same as in FreeRTOSConfig.h.
    SysCtlClockFreqSet(
    SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480,
                       SYSTEM_CLOCK);
    g_printMutex = xSemaphoreCreateMutex();

    clearScreenAndMoveCursorHome();

    const uint16_t taskStackDepth = 1000;
    const UBaseType_t priority = tskIDLE_PRIORITY + 1;

    int a = 4;

    BaseType_t result;
    result = xTaskCreate(uartTask, "uarttask", taskStackDepth, &a, priority,
                         NULL);
    assert(result == pdPASS);

    // Start the scheduler with vTaskStartScheduler
    vTaskStartScheduler();

    // We should never reach here since vTaskStartScheduler should block.
    assert(false);
    while (true)
    {
    }
}

