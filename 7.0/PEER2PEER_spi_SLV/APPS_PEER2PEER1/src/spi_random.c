/*
 * spi_random.c
 *
 * Created: 21/10/2019 11:56:26
 *  Author: MarianoF
 */ 
#include "spi_random.h"
#include "asf.h"
#include "sio2host.h"

#define		CMMND0		0b01000110 // cmmnd byte 0, divc addss 01, reg add 0x1, inc write
#define		CMMND1		0b01110110 // cmmnd byte 1, divc addss 01, reg add 0xD, inc write to unlock mem
#define		CMMND2		0b01000000 // don't care cmmnd
#define		CMMND3		0b01000001 // ADCDATA read cmmnd
#define		CMMND4		0b01000111 // Incremental read to check correct configuration of device.
#define		CONFIG0		0b11110010 // ext clock, no sensor bias, standby mode
#define		CONFIG1		0b00000000 // clk pres 1, OSR 32
#define		CONFIG2		0b10001011 // BOOST=1, GAIN=1, AZ_MUX=0
#define		CONFIG3		0b11000000 // cont conv, default crc comms check, crc comms disabled, offcal and gain cal disabled
#define		ADC_IRQ		0b00000110 // irq status, irq mod all irq on pin, inact stt logic 1, fst cmmd enabled, cnrs strt disabled
#define		MUX			0b00000001 // wont be used for now, its default value matches our need
#define		SCAN		0x0000     // wont be written, 2 bytes long
#define		OFFSETCAL	0x00	   // wont be written till deemed necessary
#define		TIMER		0x000000   //
#define		GAINCAL		0x00	   // wont be written till deemed necessary
#define		LOCK_on		0x00	   // writing any value but the correct one to this register, locks the register map of the ADC
#define		LOCK_off	0xA5	   // correct key to unlock memory map of ADC

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
/*
 * Pointer to external ADC registers 
 */
volatile uint8_t adc_reg_pointer = 0;

volatile bool read_adc_config = false;

uint8_t SPI_Transmit(uint8_t Data)
{	
	SPDR = Data;
	while(!(SPSR & (1<<SPIF))){};
}

/* Initiate SPI module */
void init_SPI_slv(void){

	sysclk_enable_module( POWER_RED_REG0, PRSPI_bm );
	
	DDR_SPI |= (1<<DD_MISO); //spi pins on port B MISO output 
	
	SPCR = ((1<<SPIE)|(1<<SPE)|(0<<DORD)|(0<<MSTR)|
	(0<<CPOL)|(0<<CPHA)|(1<<SPR1)|(1<<SPR0));  // SPI enable, Master, f/2 with TxRx OSC
	SPSR |= (1<<SPI2X); //double speed enabled
	

}

uint8_t spi2host_rx(uint8_t *data, uint8_t max_length)
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

SPI_HOST_ISR_VECT()
{
	uint8_t temp;

	temp = SPDR;
	
	/* Introducing critical section to avoid buffer corruption. */
	cpu_irq_disable();
	
	if( temp == CMMND4 )
	{
		read_adc_config = true;
	}
	if( read_adc_config )
	{
		switch (adc_reg_pointer)
		{
			case 0:
			{
				SPDR = CONFIG0;
				adc_reg_pointer++;
				break;
			}
			case 1:
			{
				SPDR = CONFIG1;
				adc_reg_pointer++;
				break;
			}
			case 2:
			{
				SPDR = CONFIG2;
				adc_reg_pointer++;
				break;
			}
			case 3:
			{
				SPDR = CONFIG3;
				adc_reg_pointer++;
				break;
			}
			case 4:
			{
				SPDR = ADC_IRQ;
				adc_reg_pointer++;
				break;
			}
			case 5:
			{
				SPDR = LOCK_off;
				break;
			}
			default:
				break;
			
			return;			
		}
	}
	

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
}
