#include "Ptimer.h"

__IO Ptimer_t Ptimer[MAX_NUMBER_OF_TIMERS] 	= {{255,0}};
__IO ptimerFlags_t ptimerFlags				= {0};

void SetPTimer(uint8_t NewNumber, uint32_t NewTime)
{
	uint8_t free_timer			= 255;
	uint8_t interrupts_enable	= ~__get_PRIMASK();

	if (interrupts_enable) __disable_irq();

	for (uint8_t i = 0; i != MAX_NUMBER_OF_TIMERS; i++)
	{
		if (Ptimer[i].Number == NewNumber)
		{
			Ptimer[i].Time = NewTime;
			free_timer = 255;
			break;
		}
		if (Ptimer[i].Number == 255)
		{
			free_timer = i;
		}
	}
	if (free_timer < MAX_NUMBER_OF_TIMERS)
	{
		Ptimer[free_timer].Number = NewNumber;
		Ptimer[free_timer].Time = NewTime;
	}

	if (interrupts_enable) __enable_irq();
}

void PTimer(void)
{
	for (uint8_t i = 0; i != MAX_NUMBER_OF_TIMERS; i++)
	{
		if (Ptimer[i].Number != 255)
		{
			if (Ptimer[i].Time != 0)
			{
				Ptimer[i].Time--;
			}
			else
			{
				ptimerFlags.timer |= 1 << Ptimer[i].Number;
				Ptimer[i].Number = 255;
			}
		}
	}
}

void InitPTimer(void)
{
	for (uint8_t i = 0; i != MAX_NUMBER_OF_TIMERS; i++)
	{
		Ptimer[i].Number = 255;
		Ptimer[i].Time = 0;
	}
}

void KillPTimer(uint8_t timerNumber)
{
	uint8_t interrupts_enable = ~__get_PRIMASK();
	if (interrupts_enable) __disable_irq();
	for (uint8_t i = 0; i != MAX_NUMBER_OF_TIMERS; i++)
	{
		if (Ptimer[i].Number == timerNumber)
		{
			Ptimer[i].Number = 255;
			Ptimer[i].Time = 0;
			break;
		}
	}
	if (interrupts_enable) __enable_irq();
}
