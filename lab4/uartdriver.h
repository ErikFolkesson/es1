#ifndef UARTDRIVER_H_
#define UARTDRIVER_H_

#include <stdint.h>

void UART_init(uint32_t base);
char UART_getChar(void);
void UART_putChar(char c);
void UART_reset(void);
void UART_putString(const char *string);
void UART_getString(char *buf, uint32_t bufSize);

#endif /* UARTDRIVER_H_ */
