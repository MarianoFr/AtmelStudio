/*
 * spi_random.h
 *
 * Created: 21/10/2019 11:56:15
 *  Author: MarianoF
 */ 
#include "asf.h"
#include "nwk.h"

#ifndef SPI_RANDOM_H_
#define SPI_RANDOM_H_

#define SPI_HOST_ISR_VECT()		  ISR(SPI_STC_vect) 

#define		DDR_SPI			DDRB
#define 	DD_MOSI			PORTB2
#define		DD_MISO			PORTB3
#define		SS				PORTB0
#define		SS2				PORTD5
#define		DD_SCK			PORTB1

#define SPI_RX_BUF_SIZE_HOST    156


/* Initiate SPI module */
void init_SPI_slv(void);

/* Receive SPI data, used to receive more than 1 byte of data*
   by not ussing the SPI interrupt, we can receive more than one byte */
void rx_Spi_16o( void );

uint8_t SPI_Transmit(uint8_t Data);

/**
 * \brief Receives data from SPI
 *
 * \param data pointer to the buffer where the received data is to be stored
 * \param max_length maximum length of data to be received
 *
 * \return actual number of bytes received
 */
uint8_t spi2host_rx(uint8_t *data, uint8_t max_length);

#endif /* SPI_RANDOM_H_ */