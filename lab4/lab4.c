#include "uartdriver.h"

#include "inc/hw_memmap.h"

#include <stdbool.h>
#include <string.h>

enum
{
    BUF_SIZE = 100
};
char buf[BUF_SIZE];

int main(void)
{
    // UART_reset();
    UART_init(UART0_BASE);
    UART_putChar('a');
    while (true)
    {
        UART_getString(buf, BUF_SIZE);
        UART_putString(buf);

        if (strcmp(buf, "end") == 0)
        {
            break;
        }
    }

    // Attempt to read after resetting driver, this should cause an error. (assert?)
    UART_reset();
    UART_getString(buf, BUF_SIZE);
}
