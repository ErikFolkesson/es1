#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>

void setupTimer(void);
void startTimer(void);
void stopTimer(void);
void resetTimer(void);
void setTimer(uint8_t hours, uint8_t minutes, uint8_t seconds);

#endif /* TIMER_H_ */
