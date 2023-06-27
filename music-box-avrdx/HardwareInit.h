#ifndef HARDWARE_INIT_H__
#define HARDWARE_INIT_H__


#define F_CPU                           (24000000UL)         /* using default clock 4MHz*/
#define USART0_BAUD_RATE(BAUD_RATE)     ((float)(64 * F_CPU / (16 * (float)BAUD_RATE)) + 0.5)

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>
#include <stdio.h>

void USARTInit(void);
void USARTSendChar(char c);
void USART0_sendString(char *str);
void TCB0Init(void);
void TAC0Init(void);
void ClockInit(void);
void ClassdInit(void);
void DACInit(void);
void DACSetValue(uint16_t value);
void ADCInit(void);
#endif

