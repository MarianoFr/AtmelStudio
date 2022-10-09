/*
 * spi_random.h
 *
 * Created: 22/10/2019 09:21:58
 *  Author: MarianoF
 */ 


#ifndef SPI_RANDOM_H_
#define SPI_RANDOM_H_

#include "asf.h"
#include "spi_random.h"

#define SPI_HOST_ISR_VECT()		  ISR(SPI_STC_vect) 

#define		DDR_SPI			DDRB
#define 	DD_MOSI			PORTB2
#define		DD_MISO			PORTB3
#define		SS				PORTB0
#define		SS2				PORTD5
#define		DD_SCK			PORTB1

#define		START_CONFIG		0b01000110 // cmmnd byte 0, divc addss 01, reg add 0x1, inc write
#define		W_LOCK_REG		0b01110110 // cmmnd byte 1, divc addss 01, reg add 0xD, inc write to unlock mem
#define		DNT_CARE		0b01000000 // don't care cmmnd

#define SPI_RX_BUF_SIZE_HOST    156


/* Initiate SPI module */
void init_SPI_mstr(void);

/* SPI transmit and receive function */
inline uint8_t SPI_Transmit_IRQ(uint8_t Data);

void config_Ziggys_ADC(void);

/*Read from SPI buffer*/
uint8_t spi_rx(uint8_t *data, uint8_t max_length);

uint8_t SPI_Transmit_no_IRQ(uint8_t Data);


#endif /* SPI_RANDOM_H_ */