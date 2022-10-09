/*
 * ADC_config.c
 *
 * Created: 20/10/2019 20:11:26
 *  Author: MarianoF
 */ 

#include <asf.h>

#define		SS				PORTB0
#define		MISO			PORTB3
#define		MOSI			PORTB2
#define		SCK				PORTB1
#define		ADC_IRQ_pin		CONFIG_EXT_INT7
#define		ADC_DR_mask		0b00000100

/*Initial configuration for the ADC*/

#define		CMMND0		0b01000110 // cmmnd byte 0, divc addss 01, reg add 0x1, inc write
#define		CMMND1		0b01110110 // cmmnd byte 1, divc addss 01, reg add 0xD, inc write to unlock mem
#define		CMMND2		0b01000000 // don't care cmmnd
#define		CMMND3		0b01000001 // ADCDATA read cmmnd
#define		CONFIG0		0b11010010 // ext clock, no sensor bias, standby mode
#define		CONFIG1		0b00000000 // clk pres 1, OSR 32
#define		CONFIG2		0b10001011 // BOOST=1, GAIN=1, AZ_MUX=0
#define		CONFIG3		0b11000000 // cont conv, default crc comms check, crc comms disabled, offcal and gain cal disabled
#define		ADC_IRQ		0b00000110 // irq status, irq mod all irq on pin, inact stt logic 1, fst cmmd enabled, cnrs strt disabled
#define		MUX			0b00000001 // wont be used for now, its default value matches our need
#define		SCAN		0x0000     // wont be written, 2 bytes long
#define		OFFSETCAL	0x00	   // wont be written till deemed necessary
#define		TIMER		0x000000   //
#define		GAINCAL		0x00	   // wont be written till deemed necessary
#define		LOCK_on		0x00	   // writing any value but the correct one to this register, lock the register map of the ADC
#define		LOCK_off	0xA5	   // correct key to unlock memory map of ADC

/* Initiate SPI module */
void init_SPI(void){

	sysclk_enable_module( POWER_RED_REG0, PRSPI_bm );
	
	DDRB = ((1<<DDB2)|(1<<DDB1)|(1<<DDB0)); //spi pins on port B MOSI, SCK, SS outputs
	//MISO input by default
	SPCR = ((0<<SPIE)|(1<<SPE)|(0<<DORD)|(1<<MSTR)|
	(0<<CPOL)|(0<<CPHA)|(0<<SPR1)|(0<<SPR0));  // SPI enable, Master, f/2 with TxRx OSC
	
	PORTB |= (1<<SS); //Put Slave Select high
}

/* SPI transmit and receive function */
uint8_t SPI_Transmit(uint8_t Data){
	
	SPDR = Data;
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}

/* Write ADC registersvia SPI */
void config_Ziggys_ADC( void ){
	
	PORTB &= !(1<<SS);			 // Put SS LOW to start transmission.
	SPI_Transmit(CMMND0);
	SPI_Transmit(CONFIG0);
	SPI_Transmit(CONFIG1);
	SPI_Transmit(CONFIG2);
	SPI_Transmit(CONFIG3);
	SPI_Transmit(ADC_IRQ);// Here we end the inc write of the ADC.
	PORTB |= (1<<SS);			 // Put SS HIGH to stop comm.
	PORTB &= !(1<<SS);			 // Restart comms as to write the pending registers.
	SPI_Transmit(CMMND1); // inc write to lock the device.
	SPI_Transmit(LOCK_on);
	PORTB |= (1<<SS);            // stop comms.
}