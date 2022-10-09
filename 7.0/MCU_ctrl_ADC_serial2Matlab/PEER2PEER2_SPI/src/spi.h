/*
 * spi.h
 *
 * Created: 20/10/2019 19:48:37
 *  Author: MarianoF
 */ 
#include <asf.h>


#ifndef SPI_H_
#define SPI_H_

void init_SPI(void);

uint8_t SPI_Transmit(uint8_t Data);



#endif /* SPI_H_ */