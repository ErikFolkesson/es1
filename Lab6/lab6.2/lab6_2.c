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
#include "drivers/buttons.h"

#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "inc/tm4c129encpdt.h"

SemaphoreHandle_t g_printMutex;
SemaphoreHandle_t g_statusSemaphore;

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

enum
{
    BUF_CAPACITY = 15
};

// +1 to make sure we always have a NUL character at the end of the buffer.
char buf[BUF_CAPACITY + 1];
int bufSize;
int totalCharacterCount;

// Prints the ANSI escape sequence to move the cursor to the home position (1,1)
void clearScreenAndMoveCursorHome(void)
{
    uartPuts("\x1B[2J");
    uartPuts("\x1B[H");
}

// NOTE: We don't handle numbers larger than 99.
void uartPrintInt(int number)
{
    char tenDigit = '0' + number / 10 % 10;
    char oneDigit = '0' + number % 10;
    UARTCharPut(UART0_BASE, tenDigit);
    UARTCharPut(UART0_BASE, oneDigit);
}

// If the call needs to be thread-safe, the caller is responsible for locking a mutex.
void printUartData(void)
{
    clearScreenAndMoveCursorHome();
    uartPuts(buf);

    if (xSemaphoreTake(g_statusSemaphore, 0))
    {
        xSemaphoreGive(g_statusSemaphore);
        uartPuts("\r\n");
        uartPrintInt(totalCharacterCount);
    }
}

// Rotates the element in the buffer one step to the left, with the first character moving to the last position.
void rotateLeft(char *buf, int bufSize)
{
    char tmp = *buf;
    for (int i = 1; i < bufSize; i++)
    {
        buf[i - 1] = buf[i];
    }
    buf[bufSize - 1] = tmp;
}

// Task that receives input, stores it in our 15-character buffer, and echoes it back.
void uartTask(void *parameters)
{
    while (true)
    {
        int32_t c = UARTCharGet(UART0_BASE);
        // We don't care to print out any non-printable characters.
        if (!isprint(c))
        {
            continue;
        }

        // While modifying the buffer, we don't want anyone to print at the same time.
        xSemaphoreTake(g_printMutex, portMAX_DELAY);

        if (bufSize == BUF_CAPACITY)
        {
            // We move all characters in the buffer one step to the left to make space for the new, appended character.
            rotateLeft(buf, bufSize);
            buf[bufSize - 1] = c;
        }
        else
        {
            buf[bufSize] = c;
            bufSize++;
        }

        totalCharacterCount++;

        printUartData();
        xSemaphoreGive(g_printMutex);
    }
}

// Task that polls the button and handles turning on the status task when it's pressed.
void buttonTask(void *parameters)
{
    while (true)
    {
        uint8_t delta;
        uint8_t state = ButtonsPoll(&delta, 0);

        if (BUTTON_PRESSED(LEFT_BUTTON, state, delta) || BUTTON_PRESSED(RIGHT_BUTTON, state, delta))
        {
            // During the ten seconds the status task should be shown, a semaphore is raised.
            xSemaphoreGive(g_statusSemaphore);

            xSemaphoreTake(g_printMutex, portMAX_DELAY);
            printUartData();
            xSemaphoreGive(g_printMutex);

            vTaskDelay(pdMS_TO_TICKS(1000 * 10));
            // Once ten seconds have passed, we lower the semaphore for the status task.
            xSemaphoreTake(g_statusSemaphore, portMAX_DELAY);

            // Since the status task should no longer be shown, we make a new call to printUartData();
            xSemaphoreTake(g_printMutex, portMAX_DELAY);
            printUartData();
            xSemaphoreGive(g_printMutex);
        }
    }
}

int main(void)
{
    ConfigureUART();
    ButtonsInit();

    // Set the system clock to the same as in FreeRTOSConfig.h.
    SysCtlClockFreqSet(
    SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480,
                       SYSTEM_CLOCK);
    g_printMutex = xSemaphoreCreateMutex();
    g_statusSemaphore = xSemaphoreCreateBinary();

    clearScreenAndMoveCursorHome();

    const uint16_t taskStackDepth = 1000;
    const UBaseType_t priority = tskIDLE_PRIORITY + 1;

    BaseType_t result;
    result = xTaskCreate(uartTask, "uarttask", taskStackDepth, NULL, priority,
                         NULL);
    assert(result == pdPASS);
    result = xTaskCreate(buttonTask, "buttontask", taskStackDepth, NULL, priority,
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

