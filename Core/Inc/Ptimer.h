#ifndef INC_PTIMER_H_
#define INC_PTIMER_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"

typedef struct
{
	uint8_t Number;
	uint32_t Time;
} Ptimer_t;

#define MAX_NUMBER_OF_TIMERS 8

typedef struct
{
	unsigned timer		:MAX_NUMBER_OF_TIMERS;
} ptimerFlags_t;

extern __IO Ptimer_t Ptimer[MAX_NUMBER_OF_TIMERS];
extern __IO ptimerFlags_t ptimerFlags;

void SetPTimer(uint8_t NewNumber, uint32_t NewTime);
void PTimer(void);
void InitPTimer(void);
void KillPTimer(uint8_t timerNumber);

#endif /* INC_PTIMER_H_ */
