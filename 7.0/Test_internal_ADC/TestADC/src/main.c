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
#include "sio2host.h"

/*- Definitions ------------------------------------------------------------*/
#define		APP_RX_BUF_SIZE		109
/*- Variables --------------------------------------------------------------*/
static uint8_t adc_rx_length;
static uint16_t adc_data[APP_RX_BUF_SIZE];/*109 max payload*/
static uint16_t adc_rx_buffer[APP_RX_BUF_SIZE];
static uint8_t adc_rx_buf_tail;
static uint8_t adc_rx_buf_head;
static uint8_t adc_rx_count;
volatile uint8_t button_down;
volatile uint8_t count = 0;
static uint8_t temp[2];
static uint16_t temperature;
static char str_temp[4];




/*************************************************************************//**
*****************************************************************************/
/*Switch Debounce script*/
static inline void debounce(void){
	// Counter for number of equal states
	static uint8_t count = 0;
	// Keeps track of current (debounced) state
	static uint8_t button_state = 0;

	// Check if button is high or low for the moment
	uint8_t current_state = (~PINE & (1<<PORTE4)) != 0;
	if(current_state != button_state) {
		// Button state is about to be changed, increase counter
		count++;
		if(count >= 4) {
			// The button have not bounced for four checks, change state
			button_state = current_state;
			// If the button was pressed (not released), tell main so
			if(current_state != 0) {
				button_down = 1;
			}
			count = 0;
		}
	}
	else{
		// Reset counter
		count = 0;
	}
}
/*************************************************************************//**
*****************************************************************************/
/* ADC sort samples into buffer*/
uint8_t adc_rx(uint16_t *data, uint8_t max_length)
{			

	uint8_t data_received = 0;
	if(adc_rx_buf_tail >= adc_rx_buf_head)
	{
		adc_rx_count = adc_rx_buf_tail - adc_rx_buf_head;
	}
	else
	{
		adc_rx_count = adc_rx_buf_tail + (APP_RX_BUF_SIZE - adc_rx_buf_head);
	}
	
	if (0 == adc_rx_count) {
		return 0;
	}

	if (APP_RX_BUF_SIZE <= adc_rx_count) {
		/*
		 * Bytes between head and tail are overwritten by new data.
		 * The oldest data in buffer is the one to which the tail is
		 * pointing. So reading operation should start from the tail.
		 */
		adc_rx_buf_head = adc_rx_buf_tail;

		/*
		 * This is a buffer overflow case. But still only the number of
		 * bytes equivalent to
		 * full buffer size are useful.
		 */
		adc_rx_count = APP_RX_BUF_SIZE;

		/* Bytes received is more than or equal to buffer. */
		if (APP_RX_BUF_SIZE <= max_length) {
			/*
			 * Requested receive length (max_length) is more than
			 * the
			 * max size of receive buffer, but at max the full
			 * buffer can be read.
			 */
			max_length = APP_RX_BUF_SIZE;
		}
	} else {
		/* Bytes received is less than receive buffer maximum length. */
		if (max_length > adc_rx_count) {
			/*
			 * Requested receive length (max_length) is more than
			 * the data
			 * present in receive buffer. Hence only the number of
			 * bytes
			 * present in receive buffer are read.
			 */
			max_length = adc_rx_count;
		}
	}

	data_received = max_length;
	while (max_length > 0) {
		/* Start to copy from head. */
		*data = adc_rx_buffer[adc_rx_buf_head];
		data++;
		max_length--;
		if ((APP_RX_BUF_SIZE - 1) == adc_rx_buf_head) {
			adc_rx_buf_head = 0;
		}
		else
		{
			adc_rx_buf_head++;
		}
	}
	return data_received;
}
static void print_samples(void)
{
	adc_rx_length = adc_rx(adc_data, APP_RX_BUF_SIZE);
	if (adc_rx_length) {
		for (uint8_t i = 0; i < adc_rx_length; i++) {
			itoa( adc_data[i], str_temp, 10 );
			for( uint8_t k = 0; k < sizeof(str_temp); k++ ){
				sio2host_putchar( str_temp[k] );
			}
			
		}
		sio2host_putchar( 10 );
	}
}
void adc_meastemp (void)
{
	/*ADCSRC = 10<<ADSUT0; // set start-up time
	ADCSRB = 1<<MUX5; // set MUX5 first
	ADMUX = (0<<ADLAR); // store new ADMUX, AVDD AREF
	adc_set_voltage_reference(ADC_VREF_1V6);
	adc_set_mux(ADC_MUX_1V2);*/
	// switch ADC on, set prescaler, start conversion
	ADCSRA |= (1<<ADSC);
	do
	{} while( (ADCSRA & (1<<ADSC))); // wait for conversion end
	ADCSRA = 0; // disable the ADC
	temp[1]=ADCL;
	temp[0]=ADCH;
	return;
}
int main (void)
{
	sysclk_init();
	board_init();
	sio2host_init();
	DDRF |= ( 0 << PORTF1 ) | ( 0 << PORTF0 );
	/*Initiates internal ADC, select temp sensor to test 
	ADC functionality. By default, cont conversions*/
	adc_init( ADC_PRESCALER_DIV16, ADC_VREF_1V6, ADC_MUX_TEMP_SENSOR, true, true );
	cpu_irq_enable();
	
	while (1)
	{
		debounce();
		if ( button_down ){
			// Clear flag
			button_down = 0;
			adc_start_conversion();
			//adc_meastemp();
			//temperature =  temp[0]*(256) + temp[1];
			
			/*itoa( temperature, str_temp, 10 );
			for( uint8_t i = 0; i < sizeof(str_temp); i++ ){
				sio2host_putchar( str_temp[i] );
			}*/
			
			/*while( count < APP_RX_BUF_SIZE-1 );
			cpu_irq_disable();
			print_samples();
			cpu_irq_enable();*/
		}
		
	}
}

/* ADC convertion complete IRQ */

ISR( ADC_vect ){

	/* Introducing critical section to avoid buffer corruption. */
	cpu_irq_disable();
	temp[1]=ADCL;
	temp[0]=ADCH;
	temperature =  temp[0]*(256) + temp[1];
	/* The number of data in the receive buffer is incremented and the
	 * buffer is updated. */
	//adc_rx_buffer[adc_rx_buf_tail] = temperature;
	//if ((APP_RX_BUF_SIZE - 1) == adc_rx_buf_tail) {
		/* Reached the end of buffer, revert back to beginning of
		 * buffer. */
		//adc_rx_buf_tail = 0x00;
	//} else {
	//	adc_rx_buf_tail++;
	//}
	/*count++;
	if(count==APP_RX_BUF_SIZE){ count=0;}*/
	/*itoa( temperature, str_temp, 10 );
	for( uint8_t k = 0; k < sizeof(str_temp); k++ ){
		sio2host_putchar( str_temp[k] );
	}*/
	sio2host_putchar( temp[0] );
	sio2host_putchar( temp[1] );
	cpu_irq_enable();
	}
	
