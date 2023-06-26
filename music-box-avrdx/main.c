/*
* music-box-avrdx.c
*
* Created: 2023/6/26 19:41:37
* Author : yuan
*/


#define F_CPU                           (24000000UL)         /* using default clock 4MHz*/
#define USART0_BAUD_RATE(BAUD_RATE)     ((float)(64 * F_CPU / (16 * (float)BAUD_RATE)) + 0.5)
#define PERIOD_EXAMPLE_VALUE			(0x0FFF)

#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>
#include <stdio.h>

void USART0_init(void);
void USART0_sendChar(char c);
void USART0_sendString(char *str);

#define TIMER_PERIOD        0x2000

ISR(TCA0_OVF_vect) {

	PORTC.OUTTGL = PIN3_bm;

	TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}

void TAC0Init(void)
{
	/* Configure LED0 pin as output */
	PORTC.DIRSET = PIN3_bm;

	TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
	TCA0.SINGLE.PER = TIMER_PERIOD;
	TCA0.SINGLE.CTRLA = (TCA_SINGLE_CLKSEL_1_bm | TCA_SINGLE_CLKSEL_2_bm);
	TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
}



void USART0_sendChar(char c)
{
	while(!(USART0.STATUS & USART_DREIF_bm))
	{
		;
	}
	
	USART0.TXDATAL = c;
}

static int uart0_putchar(char c, FILE *stream);

static FILE uart0Stdout = FDEV_SETUP_STREAM(uart0_putchar, NULL,
_FDEV_SETUP_WRITE);

static int
uart0_putchar(char c, FILE *stream)
{
	/* Wait for empty transmit buffer */
	USART0_sendChar(c);
	return 0;
}

void USART0_sendString(char *str)
{
	for(size_t i = 0; i < strlen(str); i++)
	{
		USART0_sendChar(str[i]);
	}
}
void USART0_init(void)
{
	PORTA.DIRSET = PIN0_bm;                             /* set pin 0 of PORT C (TXd) as output*/
	PORTA.DIRCLR = PIN1_bm;                             /* set pin 1 of PORT C (RXd) as input*/
	
	USART0.BAUD = (uint16_t)(USART0_BAUD_RATE(115200));   /* set the baud rate*/
	
	USART0.CTRLC = USART_CHSIZE_0_bm
	| USART_CHSIZE_1_bm;                    /* set the data format to 8-bit*/
	
	USART0.CTRLB |= USART_TXEN_bm;                      /* enable transmitter*/
	stdout = &uart0Stdout;
}

/* VREF start-up time */
#define VREF_STARTUP_TIME       (50)
/* Mask needed to get the 2 LSb for DAC Data Register */
#define LSB_MASK                (0x03)
/* Number of samples for a sine wave period */
#define SINE_PERIOD_STEPS       (100)
/* Sine wave amplitude */
#define SINE_AMPLITUDE          (511)
/* Sine wave DC offset */
#define SINE_DC_OFFSET          (512)
/* Frequency of the sine wave */
#define SINE_FREQ               (100)
/* Step delay for the loop */
#define STEP_DELAY_TIME         ((100000000 / SINE_FREQ) / SINE_PERIOD_STEPS)

static void sineWaveInit(void);
static void VREF_init(void);
static void DAC0_init(void);
static void DAC0_setVal(uint16_t value);

/* Buffer to store the sine wave samples */
uint16_t sineWave[SINE_PERIOD_STEPS];

static void sineWaveInit(void)
{
	uint8_t i;
	for(i = 0; i < SINE_PERIOD_STEPS; i++)
	{
		sineWave[i] = SINE_DC_OFFSET + SINE_AMPLITUDE * sin(2 * M_PI * i / SINE_PERIOD_STEPS);
	}
}

static void VREF_init(void)
{
	VREF.DAC0REF = VREF_REFSEL_VDD_gc /* Select the 2.048V Internal Voltage Reference for DAC */
	| VREF_ALWAYSON_bm;    /* Set the Voltage Reference in Always On mode */
	/* Wait VREF start-up time */
	_delay_us(VREF_STARTUP_TIME);
}

static void DAC0_init(void)
{
	/* Disable digital input buffer */
	PORTD.PIN6CTRL &= ~PORT_ISC_gm;
	PORTD.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	/* Disable pull-up resistor */
	PORTD.PIN6CTRL &= ~PORT_PULLUPEN_bm;
	DAC0.CTRLA = DAC_ENABLE_bm          /* Enable DAC */
	| DAC_OUTEN_bm           /* Enable output buffer */
	| DAC_RUNSTDBY_bm;       /* Enable Run in Standby mode */
}

static void DAC0_setVal(uint16_t value)
{
	/* Store the two LSbs in DAC0.DATAL */
	DAC0.DATAL = (value & LSB_MASK) << 6;
	/* Store the eight MSbs in DAC0.DATAH */
	DAC0.DATAH = value >> 2;
}


/* This function initializes the PORT module */
void PORT_init(void)
{
	
	/* Disable interrupt and digital input buffer on PD2 */
	PORTD.PIN2CTRL &= ~PORT_ISC_gm;
	PORTD.PIN2CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	
	/* Disable pull-up resistor */
	PORTD.PIN2CTRL &= ~PORT_PULLUPEN_bm;
}

/* This function initializes the VREF module */
void VREF0_init(void)
{
	VREF.ADC0REF = VREF_REFSEL_VDD_gc;  /* Internal 2.048V reference */
}

/* This function initializes the ADC module */
void ADC0_init(void)
{
	ADC0.CTRLC = ADC_PRESC_DIV4_gc;        /* CLK_PER divided by 4 */
	ADC0.CTRLA = ADC_ENABLE_bm             /* ADC Enable: enabled */
	| ADC_RESSEL_12BIT_gc;      /* 12-bit mode */
	ADC0.MUXPOS = ADC_MUXPOS_AIN2_gc;      /* Select ADC channel AIN3 <-> PD3 */

}
void ADC0_SelctChn(ADC_MUXPOS_t chn)
{
	ADC0.MUXPOS = chn;      /* Select ADC channel AIN3 <-> PD3 */
}

/* This function starts the ADC conversions*/
void ADC0_start(void)
{
	/* Start ADC conversion */
	ADC0.COMMAND = ADC_STCONV_bm;
}
/* This function returns the ADC conversion result */
uint16_t ADC0_read(void)
{
	/* Wait for ADC result to be ready */
	while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
	/* Clear the interrupt flag by reading the result */
	return ADC0.RES;
}


int main(void)
{
	uint8_t sineIndex = 0;
	ccp_write_io((void *)&(CLKCTRL.OSCHFCTRLA),CLKCTRL_FRQSEL_24M_gc);
	PORTC.DIRSET = PIN0_bm;
	PORTC.OUTSET = PIN0_bm;
	TAC0Init();
	VREF_init();
	DAC0_init();
	PORT_init();
	VREF0_init();
	ADC0_init();
	sineWaveInit();
	USART0_init();
	sei();

	
	while (1)
	{
		//DAC0_setVal(sineWave[sineIndex++]);
		//if(sineIndex == SINE_PERIOD_STEPS)
		//sineIndex = 0;
		//_delay_us(STEP_DELAY_TIME);
		ADC0_SelctChn(ADC_MUXPOS_AIN1_gc);
		ADC0_start();
		uint16_t potv = ADC0_read();
		/* Transmit the ADC result to be plotted using Data Visualizer */
		printf("ADC VUSB:%u",potv);
		ADC0_SelctChn(ADC_MUXPOS_AIN2_gc);
		ADC0_start();
		potv = ADC0_read();
		/* Transmit the ADC result to be plotted using Data Visualizer */
		printf("ADC POT:%u",potv);
		_delay_ms(1000);
	}
}