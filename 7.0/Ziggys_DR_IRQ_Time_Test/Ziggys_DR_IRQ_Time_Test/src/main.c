/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "led.h"
#include "sysclk.h"
#include "sio2host.h"
#include "asf.h"
#include "spi_random.h"
#include "conf_sio2host.h"

#define		DNT_CARE		0b01000000 // don't care cmmnd.
#define		R_ADC_SAMPLES	0b01000001 // ADCDATA read cmmnd.
#define		ADC_STAT_DR		0b00010011 // ADC Status Byte DR Mask

static uint8_t number_of_samples = 0;
uint8_t LADCs;
uint8_t HADCs;
uint8_t MADCs;
uint8_t STATUS_byte = 0;
bool sample_ready;

int main (void)
{
	sample_ready = 0;
	
	while(!UCSR1A & USART_DRE_bm);
	UDR1 = HADCs;
	while(!UCSR1A & USART_DRE_bm);
	UDR1 = MADCs;
	while(!UCSR1A & USART_DRE_bm);
	UDR1 = LADCs;
	
	cpu_irq_disable();
	/*Send don't care command to ADC to retrieve STATUS BYTE */
	PORTD &= ~ ( 1 << SS2 ); // Put Slave Select low to start transmission
	SPDR = DNT_CARE;
	while(!(SPSR & (1<<SPIF))){};
	STATUS_byte = SPDR;
	PORTD |= ( 1 << SS2 );					  // SDO pin in the slave transmits its STATUS BYTE.
	if ( STATUS_byte == ADC_STAT_DR )         
	{
		PORTD &= ~( 1 << SS2 );
		SPDR = R_ADC_SAMPLES;
		while(!(SPSR & (1<<SPIF))){};
		STATUS_byte = SPDR;
		SPDR = DNT_CARE;
		while(!(SPSR & (1<<SPIF))){};
		HADCs = SPDR;
		SPDR = DNT_CARE;
		while(!(SPSR & (1<<SPIF))){};
		MADCs = SPDR;
		SPDR = DNT_CARE;
		while(!(SPSR & (1<<SPIF))){};
		LADCs = SPDR;
		PORTD |= ( 1 << SS2 );
		while(!UCSR1A & USART_DRE_bm);
		UDR1 = HADCs;
		while(!UCSR1A & USART_DRE_bm);
		UDR1 = MADCs;
		while(!UCSR1A & USART_DRE_bm);
		UDR1 = LADCs;
		sample_ready = 1;

	}
	cpu_irq_enable();
}
