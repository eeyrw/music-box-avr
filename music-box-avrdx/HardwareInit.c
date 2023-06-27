/*
* HardwareInit.c
*
* Created: 6/27/2023 12:41:04 PM
*  Author: uie38447
*/

#include "HardwareInit.h"


void TCB0Init(void)
{
	TCB0.CCMP = F_CPU/2/32000; //32khz
	TCB0.CTRLA |= TCB_ENABLE_bm;
	TCB0.CTRLA |= TCB_CLKSEL_DIV2_gc;
	TCB0.CTRLB = TCB_CNTMODE_INT_gc;
	TCB0.INTCTRL = TCB_CAPT_bm;

}
void TAC0Init(void)
{
	/* Configure LED0 pin as output */
	PORTC.DIRSET = PIN3_bm;
    /* set waveform output on PORT A */
    PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTC_gc;
    
    /* enable split mode */
    TCA0.SPLIT.CTRLD = TCA_SPLIT_SPLITM_bm;
    
    TCA0.SPLIT.CTRLB = TCA_SPLIT_HCMP0EN_bm;        /* enable compare channel 0 for the higher byte */
    
    /* set the PWM frequencies and duty cycles */
    TCA0.SPLIT.HPER = 0xff;
    TCA0.SPLIT.HCMP0 = 0x00;
    
    TCA0.SPLIT.CTRLA = TCA_SPLIT_CLKSEL_DIV2_gc    /* set clock source (sys_clk/16) */
    | TCA_SPLIT_ENABLE_bm;         /* start timer */
}


static int uart0_putchar(char c, FILE *stream);
static FILE uart0Stdout = FDEV_SETUP_STREAM(uart0_putchar, NULL,
_FDEV_SETUP_WRITE);

static int
uart0_putchar(char c, FILE *stream)
{
	/* Wait for empty transmit buffer */
	USARTSendChar(c);
	return 0;
}

void USARTSendChar(char c)
{
	while(!(USART0.STATUS & USART_DREIF_bm))
	{
		;
	}
	USART0.TXDATAL = c;
}


void ClockInit(void)
{
	ccp_write_io((void *)&(CLKCTRL.OSCHFCTRLA),CLKCTRL_FRQSEL_24M_gc);
}

void ClassdInit(void)
{
	PORTC.DIRSET = PIN0_bm;
	PORTC.OUTSET = PIN0_bm;
}

void USARTInit(void)
{
	PORTA.DIRSET = PIN0_bm;                             /* set pin 0 of PORT C (TXd) as output*/
	PORTA.DIRCLR = PIN1_bm;                             /* set pin 1 of PORT C (RXd) as input*/
	
	USART0.BAUD = (uint16_t)(USART0_BAUD_RATE(115200));   /* set the baud rate*/
	
	USART0.CTRLC = USART_CHSIZE_0_bm
	| USART_CHSIZE_1_bm;                    /* set the data format to 8-bit*/
	
	USART0.CTRLB |= USART_TXEN_bm;                      /* enable transmitter*/
	stdout = &uart0Stdout;
}

void DACInit(void)
{
	VREF.DAC0REF = VREF_REFSEL_VDD_gc /* Select the 2.048V Internal Voltage Reference for DAC */
	| VREF_ALWAYSON_bm;    /* Set the Voltage Reference in Always On mode */
	/* Wait VREF start-up time */
	_delay_us(50);
	
	/* Disable digital input buffer */
	PORTD.PIN6CTRL &= ~PORT_ISC_gm;
	PORTD.PIN6CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	
	/* Disable pull-up resistor */
	PORTD.PIN6CTRL &= ~PORT_PULLUPEN_bm;
	DAC0.CTRLA = DAC_ENABLE_bm          /* Enable DAC */
	| DAC_OUTEN_bm           /* Enable output buffer */
	| DAC_RUNSTDBY_bm;       /* Enable Run in Standby mode */
	DACSetValue(0);
}

void DACSetValue(uint16_t value)
{
	/* Store the two LSbs in DAC0.DATAL */
	//DAC0_DATAL = (value & 0x03) << 6;
	/* Store the eight MSbs in DAC0.DATAH */
	//DAC0_DATAH = value >> 2;
	DAC0_DATA = value<<6;
}

/* This function initializes the ADC module */
void ADCInit(void)
{
	/* Disable interrupt and digital input buffer on PD2 */
	PORTD.PIN2CTRL &= ~PORT_ISC_gm;
	PORTD.PIN2CTRL |= PORT_ISC_INPUT_DISABLE_gc;
	
	/* Disable pull-up resistor */
	PORTD.PIN2CTRL &= ~PORT_PULLUPEN_bm;
	
	VREF.ADC0REF = VREF_REFSEL_VDD_gc;  /* Internal 2.048V reference */
	ADC0.CTRLC = ADC_PRESC_DIV4_gc;        /* CLK_PER divided by 4 */
	ADC0.CTRLA = ADC_ENABLE_bm             /* ADC Enable: enabled */
	| ADC_RESSEL_12BIT_gc;      /* 12-bit mode */
	ADC0.MUXPOS = ADC_MUXPOS_AIN2_gc;      /* Select ADC channel AIN3 <-> PD3 */
}


/* This function returns the ADC conversion result */
uint16_t ADCRead(ADC_MUXPOS_t chn)
{
	ADC0.MUXPOS = chn;      /* Select ADC channel AIN3 <-> PD3 */
	/* Start ADC conversion */
	ADC0.COMMAND = ADC_STCONV_bm;
	/* Wait for ADC result to be ready */
	while (!(ADC0.INTFLAGS & ADC_RESRDY_bm));
	/* Clear the interrupt flag by reading the result */
	return ADC0.RES;
}