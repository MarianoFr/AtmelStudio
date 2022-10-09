/**
 * Use of the Internal to the MCU ADC
 * sending samples via USART to MATLAB
 *
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#include "sio2host.h"
#include <string.h>
#include "conf_sio2host.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "adc.c"
/*Definitions*/
#define		ADC_IDLE		0
#define		ADC_CONVERTING	1		

/* Variables */
static uint8_t rx_cmnd[9];
volatile uint8_t cmnd_length = 9;
volatile uint8_t temp[2]; 
volatile uint16_t temperature;
volatile uint8_t button_down;
uint8_t rx_char = 0;
uint8_t prev_char = 0;
uint8_t cnt = 0;
uint32_t number_of_samples = 666660UL;
uint16_t i=0;
uint16_t j=0;
uint16_t f0 = 640;
float t0=0;
uint32_t fs=0;//sampling frequency
char cmnd1[9] = "transmit";
char str_temp[4];
bool converting;
uint8_t board_status = ADC_IDLE;
char float2string[4];
char tx_buf[38];
uint8_t tx_length = sizeof(tx_buf);

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

int main (void)
{
	/*Insert system clock initialization code here (sysclk_init()).*/

	sysclk_init();
	board_init();
	sio2host_init();
	/*Initiates internal ADC, select temp sensor to test ADC functionality. By default, cont conversions*/
	fs = adc_init( ADC_PRESCALER_DIV16, ADC_VREF_1V5, ADC_MUX_ADC0, true, true, ADC_AUTOTRIGGER_SOURCE_FREERUNNING );
	cpu_irq_enable();
	sprintf( tx_buf, "La tasa de muestreo es de %lu", fs );
	/*Disable USART interrupts*/
	usart_tx_complete_interrupt_disable(USART_HOST);
	usart_rx_complete_interrupt_disable(USART_HOST);
	usart_data_empty_interrupt_disable(USART_HOST);
		
	while(1){//Infinite loop
		
		debounce();//check on board switch status
		while(board_status == ADC_IDLE){//if device not converting, receive instructions
		
			rx_char = usart_getchar( USART_HOST );
		
			if( rx_char != prev_char ){//Analyze if received instruction 
				rx_cmnd[cnt] = rx_char;//matches any of the defined ones
				usart_putchar(USART_HOST, rx_char);
				if( rx_cmnd[cnt] == cmnd1[cnt] ) 
					cnt++;									
			}
			if( cnt == 8 ){
				if( strcmp( (char *) rx_cmnd, cmnd1 ) == 0 ){
					LED_On(LED0);
					memset(rx_cmnd, 0, sizeof(rx_cmnd));
					cnt = 0 ;
					board_status = ADC_CONVERTING;
					adc_start_conversion();
				}
			}
		}
		prev_char = rx_char;
		if ( button_down ){
				LED_Off(LED0);
				adc_disable();
				adc_enable();
				button_down = 0;
			}
		
	}
}

/* ADC conversion complete IRQ */

ISR( ADC_vect ){

	/* Introducing critical section to avoid buffer corruption. */
	//cpu_irq_disable();
	//temp[1]=ADCL;
	temp[0]=ADCH;
	/*temperature =  temp[0]*256 + temp[1];*/
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
	if(count==APP_RX_BUF_SIZE){ count=0;}
	itoa( temperature, str_temp, 10 );
	for( uint8_t k = 0; k < sizeof(str_temp); k++ ){
		sio2host_putchar( str_temp[k] );
	}
	sio2host_putchar('\n');*/
	
	//sio2host_putchar( temp[1] );
	
	//number_of_samples--;
	
	sio2host_putchar( temp[0] );
	
	
	/*if( number_of_samples == 0 )
	{
		LED_Off(LED0);
		adc_disable();
		adc_enable();
		number_of_samples = 666660UL;
	}*/
	/*cpu_irq_enable()*/;	
}