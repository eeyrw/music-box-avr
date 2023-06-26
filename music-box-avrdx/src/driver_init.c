/**
 * \file
 *
 * \brief Driver initialization.
 *
 (c) 2020 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms,you may use this software and
    any derivatives exclusively with Microchip products.It is your responsibility
    to comply with third party license terms applicable to your use of third party
    software (including open source software) that may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 */

/*
 * Code generated by START.
 *
 * This file will be overwritten when reconfiguring your START project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <system.h>

/* configure the pins and initialize the registers */
void USART_0_initialization(void)
{

	// Set pin direction to input
	PA1_set_dir(PORT_DIR_IN);

	PA1_set_pull_mode(
	    // <y> Pull configuration
	    // <id> pad_pull_config
	    // <PORT_PULL_OFF"> Off
	    // <PORT_PULL_UP"> Pull-up
	    PORT_PULL_OFF);

	// Set pin direction to output

	PA0_set_level(
	    // <y> Initial level
	    // <id> pad_initial_level
	    // <false"> Low
	    // <true"> High
	    false);

	PA0_set_dir(PORT_DIR_OUT);

	USART_0_init();
}

/* configure pins and initialize registers */
void DAC_0_initialization(void)
{

	// Disable digital input buffer
	PD6_set_isc(PORT_ISC_INPUT_DISABLE_gc);
	// Disable pull-up resistor
	PD6_set_pull_mode(PORT_PULL_OFF);

	DAC_0_init();
}

/* Configure pins and initialize registers */
void ADC_0_initialization(void)
{

	ADC_0_init();
}

void TIMER_0_initialization(void)
{

	TIMER_0_init();
}

void PWM_0_initialization(void)
{

	// Set pin direction to output

	PA3_set_level(
	    // <y> Initial level
	    // <id> pad_initial_level
	    // <false"> Low
	    // <true"> High
	    false);

	PA3_set_dir(PORT_DIR_OUT);

	PWM_0_init();
}

/**
 * \brief System initialization
 */
void system_init()
{
	mcu_init();

	CLKCTRL_init();

	USART_0_initialization();

	SLPCTRL_init();

	CPUINT_init();

	DAC_0_initialization();

	ADC_0_initialization();

	TIMER_0_initialization();

	PWM_0_initialization();

	BOD_init();
}
