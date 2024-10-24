#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "driverlib/gpio.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"

#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "inc/tm4c129encpdt.h"

#define WAIT_ITERATIONS 10000000

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
    xSemaphoreTake(g_printMutex, portMAX_DELAY);
    const char *it;
    for (it = str; *it != '\0'; it++)
    {
        UARTCharPut(UART0_BASE, *it);
    }
    xSemaphoreGive(g_printMutex);
}

typedef struct
{
    uint8_t *data;
    uint16_t size;
    uint16_t capacity;
} Buffer;

typedef struct
{
    SemaphoreHandle_t *emptySlots;
    SemaphoreHandle_t *filledSlots;
    SemaphoreHandle_t *bufferMutex;
    Buffer *buffer;
} Args;

uint8_t produceByte(void)
{
    // TODO: Visualize with UART.
    return '|';
}

void putByteIntoBuffer(Buffer *buffer, uint8_t byte)
{
    assert(buffer->size <= buffer->capacity);
    buffer->data[buffer->size] = byte;
}

uint8_t removeByteFromBuffer(Buffer *buffer)
{
    return buffer->data[buffer->size - 1];
}

void sleep(bool *wakeupCall)
{
    *wakeupCall = false;
    // Spin until the bool changes from elsewhere.
    while (!*wakeupCall)
    {
    }
}

void wakeup(bool *wakeupCall)
{
    *wakeupCall = true;
}

void producer(void *parameters)
{
    const Args *args = parameters;

    while (true)
    {
        uartPuts("Producer at start of while!\r\n");
        uint8_t byte = produceByte();
        assert(args->buffer->size <= args->buffer->capacity);

        xSemaphoreTake(args->emptySlots, portMAX_DELAY);

        assert(args->buffer->size <= args->buffer->capacity);

        xSemaphoreTake(args->bufferMutex, portMAX_DELAY);

        assert(args->buffer->size <= args->buffer->capacity);

        putByteIntoBuffer(args->buffer, byte);
        uint16_t oldSize = args->buffer->size;
//        taskYIELD();
        uint16_t newSize = args->buffer->size;
        assert(oldSize == newSize);
        args->buffer->size = oldSize + 1;

        assert(args->buffer->size <= args->buffer->capacity);
        xSemaphoreGive(args->bufferMutex);
        assert(args->buffer->size <= args->buffer->capacity);

        xSemaphoreGive(args->filledSlots);
        assert(args->buffer->size <= args->buffer->capacity);

        assert(args->buffer->size <= args->buffer->capacity);

        uartPuts("Producer at end of while!\r\n");
    }
}

void consumeByte(uint8_t byte)
{
    // TODO: Visualize with UART.
    return;
}

void consumer(void *parameters)
{
    const Args *args = parameters;

    while (true)
    {
        uartPuts("Consumer at start of while!\r\n");
        assert(args->buffer->size <= args->buffer->capacity);
        xSemaphoreTake(args->filledSlots, portMAX_DELAY);
        assert(args->buffer->size <= args->buffer->capacity);
        xSemaphoreTake(args->bufferMutex, portMAX_DELAY);
        assert(args->buffer->size <= args->buffer->capacity);
        uint8_t byte = removeByteFromBuffer(args->buffer);

        uint16_t oldSize = args->buffer->size;
        taskYIELD();
        uint16_t newSize = args->buffer->size;
        assert(oldSize == newSize);
        args->buffer->size = oldSize - 1;
        assert(args->buffer->size <= args->buffer->capacity);
        xSemaphoreGive(args->bufferMutex);
        assert(args->buffer->size <= args->buffer->capacity);
        xSemaphoreGive(args->emptySlots);

        assert(args->buffer->size <= args->buffer->capacity);

        consumeByte(byte);

        uartPuts("Consumer at end of while!\r\n");
    }
}

// Prints the ANSI escape sequence to move the cursor to the home position (1,1)
void clearScreenAndMoveCursorHome(void)
{
    uartPuts("\x1B[2J");
    uartPuts("\x1B[H");
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

    uartPuts("\r\nStarting new run\r\n");

    const uint16_t taskStackDepth = 1000;
    const UBaseType_t priority = tskIDLE_PRIORITY + 1;

    enum
    {
        BUFFER_CAPACITY = 100
    };
    uint8_t bufferData[BUFFER_CAPACITY] = { 0 };
    static Buffer buffer;
    buffer = (Buffer ) { .data = bufferData, .capacity = BUFFER_CAPACITY };
    static SemaphoreHandle_t emptySlots;
    emptySlots = xSemaphoreCreateCounting(
            BUFFER_CAPACITY, BUFFER_CAPACITY);
    static SemaphoreHandle_t filledSlots;
    filledSlots = xSemaphoreCreateCounting(
            BUFFER_CAPACITY, 0);
    static SemaphoreHandle_t bufferMutex;
    bufferMutex = xSemaphoreCreateMutex();

    // Construct task arguments as static variables to make sure they outlive the tasks.
    static Args args;
    args = (Args )
            { .buffer = &buffer, .emptySlots = emptySlots, .filledSlots =
                      filledSlots,
              .bufferMutex = bufferMutex };

    BaseType_t result;
    result = xTaskCreate(consumer, "consumer", taskStackDepth, &args, priority,
                         NULL);
    assert(result == pdPASS);

    result = xTaskCreate(producer, "producer", taskStackDepth, &args, priority,
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

