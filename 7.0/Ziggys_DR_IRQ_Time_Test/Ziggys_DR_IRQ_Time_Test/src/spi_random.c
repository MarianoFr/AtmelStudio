/*
 * spi_random.c
 *
 * Created: 22/10/2019 09:21:46
 *  Author: MarianoF
 */ 
#include "asf.h"
#include "spi_random.h"

/*Variables*/

/*Give notice of spi data stored in buffer*/
volatile bool spi_buf_complete;

/**
 * Receive buffer
 * The buffer size is defined in spi_random.h
 */
static uint8_t spi_rx_buf[SPI_RX_BUF_SIZE_HOST];
/**
 * Receive buffer tail
 */
static uint8_t spi_rx_buf_tail;
/**
 * Receive buffer head
 */
static uint8_t spi_rx_buf_head;
/**
 * Number of bytes in receive buffer
 */
static uint8_t spi_rx_count;

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

/* SPI transmit and receive function */
uint8_t SPI_Transmit_IRQ(uint8_t Data)
{	
	spi_buf_complete = 0;
	SPDR = Data;
	while(!spi_buf_complete){};
}
inline uint8_t SPI_Transmit_no_IRQ(uint8_t Data)
{	
	SPDR = Data;
	//while(!(SPSR & (1<<SPIF))){};
	return SPDR;
}
uint8_t spi_rx(uint8_t *data, uint8_t max_length)
{
	uint8_t data_received = 0;
	if(spi_rx_buf_tail >= spi_rx_buf_head)
	{
		spi_rx_count = spi_rx_buf_tail - spi_rx_buf_head;
	}
	else
	{
		spi_rx_count = spi_rx_buf_tail + (SPI_RX_BUF_SIZE_HOST - spi_rx_buf_head);
	}
	
	if (0 == spi_rx_count) {
		return 0;
	}

	if (SPI_RX_BUF_SIZE_HOST <= spi_rx_count) {
		/*
		 * Bytes between head and tail are overwritten by new data.
		 * The oldest data in buffer is the one to which the tail is
		 * pointing. So reading operation should start from the tail.
		 */
		spi_rx_buf_head = spi_rx_buf_tail;

		/*
		 * This is a buffer overflow case. But still only the number of
		 * bytes equivalent to
		 * full buffer size are useful.
		 */
		spi_rx_count = SPI_RX_BUF_SIZE_HOST;

		/* Bytes received is more than or equal to buffer. */
		if (SPI_RX_BUF_SIZE_HOST <= max_length) {
			/*
			 * Requested receive length (max_length) is more than
			 * the
			 * max size of receive buffer, but at max the full
			 * buffer can be read.
			 */
			max_length = SPI_RX_BUF_SIZE_HOST;
		}
	} else {
		/* Bytes received is less than receive buffer maximum length. */
		if (max_length > spi_rx_count) {
			/*
			 * Requested receive length (max_length) is more than
			 * the data
			 * present in receive buffer. Hence only the number of
			 * bytes
			 * present in receive buffer are read.
			 */
			max_length = spi_rx_count;
		}
	}

	data_received = max_length;
	while (max_length > 0) {
		/* Start to copy from head. */
		*data = spi_rx_buf[spi_rx_buf_head];
		data++;
		max_length--;
		if ((SPI_RX_BUF_SIZE_HOST - 1) == spi_rx_buf_head) {
			spi_rx_buf_head = 0;
		}
		else
		{
			spi_rx_buf_head++;
		}
	}
	return data_received;
}
/*************************************************************************//**
	Interrupt handler for SPI data received
*****************************************************************************/

SPI_HOST_ISR_VECT(){
	uint8_t temp;

	temp = SPDR;

	/* Introducing critical section to avoid buffer corruption. */
	cpu_irq_disable();

	/* The number of data in the receive buffer is incremented and the
	 * buffer is updated. */

	spi_rx_buf[spi_rx_buf_tail] = temp;

	if ((SPI_RX_BUF_SIZE_HOST - 1) == spi_rx_buf_tail) {
		/* Reached the end of buffer, revert back to beginning of
		 * buffer. */
		spi_rx_buf_tail = 0x00;
	} else {
		spi_rx_buf_tail++;
	}

	cpu_irq_enable();
	spi_buf_complete = 1;
}

