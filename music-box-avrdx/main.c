/*
* music-box-avrdx.c
*
* Created: 2023/6/26 19:41:37
* Author : yuan
*/

#include "HardwareInit.h"
#include "Player.h"
#include <stdio.h>


extern const unsigned char Score[];
Player mainPlayer;

void VisualizeSound(void)
{
	int16_t t;
	t = synthForAsm.mixOut;
	if (t < 0)
	t = -t;
	TCA0.SPLIT.HCMP0 = t & 0xff;
}

int main(void)
{
	ClockInit();
	ClassdInit();
	USARTInit();
	TAC0Init();
	TCB0Init();
	DACInit();
	ADCInit();
	
	PlayerInit(&mainPlayer, &synthForAsm);
	PlayerPlay(&mainPlayer, (uint8_t *)Score);
	
	sei();
	
	while(1)
	{
		PlayerProcess(&mainPlayer);
		VisualizeSound();
	}
}