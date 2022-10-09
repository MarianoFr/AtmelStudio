/*
 * spi.h
 *
 * Created: 09/01/2020 10:24:55
 *  Author: MarianoF
 */ 

#ifndef SPI_H_
#define SPI_H_

#include "asf.h"
#include "spi.h"

#define SPI_HOST_ISR_VECT()		  ISR(SPI_STC_vect)

#define		DDR_SPI			DDRB
#define 	DD_MOSI			PORTB2
#define		DD_MISO			PORTB3
#define		SS				PORTB0
#define		SS2				PORTD5
#define		DD_SCK			PORTB1



#define SPI_RX_BUF_SIZE_HOST    2000


/* Initiate SPI module */
void init_SPI_mstr(void);

/* SPI transmit and receive function */
uint8_t SPI_Transmit_IRQ(uint8_t Data);

void config_Ziggys_ADC(void);


uint8_t SPI_Transmit_no_IRQ(uint8_t Data);


#endif /* SPI_H_ */