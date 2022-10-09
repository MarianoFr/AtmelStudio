/*
 * adc.c
 *	Functions to be used with the ADC
 *
 * Created: 26/11/2019 13:41:26
 *  Author: MarianoF
 */ 
#include "adc.h"
#include "compiler.h"
#include "parts.h"
#include "sysclk.h"
#include "math.h"

/*ADC buffer size*/
#define		ADC_BUF_SIZE		109//Equals the NWK_MAX_PAYLOAD_SIZE

/**
 * Receive buffer
 * The buffer size is defined above
 */
static uint8_t adc_buf[ADC_BUF_SIZE];

/**
 * Receive buffer head
 */
static uint8_t adc_buf_head;

/**
 * Receive buffer tail
 */
static uint8_t adc_buf_tail;

/**
 * Number of bytes in adc buffer
 */
static uint8_t adc_count;

/*Auxiliary variable to receive ADC samples*/
uint8_t adc_temp;

/*Number of samples to send*/
int8_t number_samples = 10;

uint8_t read_adc_buff(uint8_t *data, uint8_t max_length)
{
	uint8_t data_received = 0;
	if(adc_buf_tail >= adc_buf_head)
	{
		adc_count = adc_buf_tail - adc_buf_head;
	}
	else
	{
		adc_count = adc_buf_tail + (ADC_BUF_SIZE - adc_buf_head);
	}
	
	if (0 == adc_count) {
		return 0;
	}

	if (ADC_BUF_SIZE <= adc_count) {
		/*
		 * Bytes between head and tail are overwritten by new data.
		 * The oldest data in buffer is the one to which the tail is
		 * pointing. So reading operation should start from the tail.
		 */
		adc_buf_head = adc_buf_tail;

		/*
		 * This is a buffer overflow case. But still only the number of
		 * bytes equivalent to
		 * full buffer size are useful.
		 */
		adc_count = ADC_BUF_SIZE;

		/* Bytes received is more than or equal to buffer. */
		if (ADC_BUF_SIZE <= max_length) {
			/*
			 * Requested receive length (max_length) is more than
			 * the
			 * max size of receive buffer, but at max the full
			 * buffer can be read.
			 */
			max_length = ADC_BUF_SIZE;
		}
	} else {
		/* Bytes received is less than receive buffer maximum length. */
		if (max_length > adc_count) {
			/*
			 * Requested receive length (max_length) is more than
			 * the data
			 * present in receive buffer. Hence only the number of
			 * bytes
			 * present in receive buffer are read.
			 */
			max_length = adc_count;
		}
	}

	data_received = max_length;
	while (max_length > 0) {
		/* Start to copy from head. */
		*data = adc_buf[adc_buf_head];
		data++;
		max_length--;
		if ((ADC_BUF_SIZE - 1) == adc_buf_head) {
			adc_buf_head = 0;
		}
		else
		{
			adc_buf_head++;
		}
	}
	return data_received;
}

ISR( ADC_vect ){
	/*number_samples--;
	if( number_samples <= 0 )
	{
		adc_disable();
	}*/
	/* Introducing critical section to avoid buffer corruption. */
	cpu_irq_disable();
	
	adc_temp = ADCH;

	/* The number of data in the receive buffer is incremented and the
	 * buffer is updated. */

	adc_buf[adc_buf_tail] = adc_temp;

	if ((ADC_BUF_SIZE - 1) == adc_buf_tail) {
		/* Reached the end of buffer, revert back to beginning of
		 * buffer. */
		adc_buf_tail = 0x00;
	} else {
		adc_buf_tail++;
	}

	cpu_irq_enable();

}
