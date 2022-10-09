/*
 * spi_random.c
 *
 * Created: 22/10/2019 09:21:46
 *  Author: MarianoF
 */ 
#include "asf.h"
#include "spi.h"


/* Initiate SPI module */
void init_SPI_mstr(void){

	sysclk_enable_module( POWER_RED_REG0, PRSPI_bm );
	
	/* Set MOSI and SCK output, all others input */
	DDR_SPI |= (1<<DD_MOSI)|(1<<DD_SCK);
	/*	No end of tx irq, enable SPI, Master, set clock rate fck/2, dord MSB first, */
	SPCR |= ((0<<SPIE)|(1<<SPE)|(0<<DORD)|(1<<MSTR)|
	(0<<CPOL)|(0<<CPHA)|(0<<SPR1)|(0<<SPR0));  // SPI enable, Master, f/2 with TxRx OSC
	SPSR |= (0<<SPI2X); //double speed enabled
	DDRD = (1<<DDD5);//SS outputs
	
	PORTD |= (1<<SS2); //Put Slave Select high
	
}

uint8_t SPI_Transmit_no_IRQ(uint8_t Data)
{	
	SPDR = Data;
	while(!(SPSR & (1<<SPIF))){};
	return SPDR;
}


