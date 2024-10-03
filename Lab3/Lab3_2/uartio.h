#ifndef UARTIO_H_
#define UARTIO_H_

#include <stdint.h>

void ConfigureUART(void);
const char* uartGetInputBuf(void);
int32_t uartGetChar(void);
void moveCursorHome(void);
void moveCursorToInputPos(void);
void eraseLineAndReturnCarriage(void);
void printClock(uint32_t counter);
void uartPuts(const char *string);
void uartEraseChar(void);

#endif /* UARTIO_H_ */
