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

/*- Includes ---------------------------------------------------------------*/
#include <asf.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "sys.h"
#if SAMD || SAMR21 || SAML21
#include "system.h"
#else
#include "led.h"
#include "sysclk.h"
#endif
#include "phy.h"
#include "nwk.h"
#include "sysTimer.h"
#include "sio2host.h"
#include "asf.h"

/*- Definitions ------------------------------------------------------------*/
#ifdef NWK_ENABLE_SECURITY
#define APP_BUFFER_SIZE     (NWK_MAX_PAYLOAD_SIZE - NWK_SECURITY_MIC_SIZE)
#else
#define APP_BUFFER_SIZE     NWK_MAX_PAYLOAD_SIZE
#endif

/*- Types ------------------------------------------------------------------*/
typedef enum AppState_t {
	APP_STATE_INITIAL,
	APP_STATE_IDLE,
} AppState_t;

/*- Prototypes -------------------------------------------------------------*/
static void appSendData(void);
uint8_t adc_rx(uint8_t *data, uint8_t max_length);

/*- Variables --------------------------------------------------------------*/
static AppState_t appState = APP_STATE_INITIAL;
static SYS_Timer_t appTimer;
static NWK_DataReq_t appDataReq;
static bool appDataReqBusy = false;
static uint8_t appDataReqBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBufferPtr = 0;
static uint8_t adc_rx_length;
static uint8_t adc_data[APP_RX_BUF_SIZE];/*109 max payload*/
static uint8_t adc_rx_buffer[APP_RX_BUF_SIZE];
static uint8_t adc_rx_buf_tail;
static uint8_t adc_rx_buf_head;
static uint8_t adc_rx_count;

/*- Implementations --------------------------------------------------------*/

/*************************************************************************//**
*****************************************************************************/
static void appDataConf(NWK_DataReq_t *req)
{
	appDataReqBusy = false;
	(void)req;
}

/*************************************************************************//**
*****************************************************************************/
static void appSendData(void)
{
	if (appDataReqBusy || 0 == appUartBufferPtr) {
		return;
	}

	memcpy(appDataReqBuffer, appUartBuffer, appUartBufferPtr);

	appDataReq.dstAddr = 1 - APP_ADDR;
	appDataReq.dstEndpoint = APP_ENDPOINT;
	appDataReq.srcEndpoint = APP_ENDPOINT;
	appDataReq.options = NWK_OPT_ENABLE_SECURITY;
	appDataReq.data = appDataReqBuffer;
	appDataReq.size = appUartBufferPtr;
	appDataReq.confirm = appDataConf;
	NWK_DataReq(&appDataReq);

	appUartBufferPtr = 0;
	appDataReqBusy = true;
	//LED_Toggle(LED0);
}

/*************************************************************************//**
*****************************************************************************/
static void appTimerHandler(SYS_Timer_t *timer)
{
	appSendData();
	(void)timer;
}

/*************************************************************************//**
*****************************************************************************/
static bool appDataInd(NWK_DataInd_t *ind)
{
	for (uint8_t i = 0; i < ind->size; i++) {
		sio2host_putchar(ind->data[i]);
	}
	LED_Toggle(LED0);
	return true;
}
/*************************************************************************//**
*****************************************************************************/
static void appInit(void)
{
	NWK_SetAddr(APP_ADDR);
	NWK_SetPanId(APP_PANID);
	PHY_SetChannel(APP_CHANNEL);
	
	PHY_SetRxState(true);

	NWK_OpenEndpoint(APP_ENDPOINT, appDataInd);

	appTimer.interval = APP_FLUSH_TIMER_INTERVAL;
	appTimer.mode = SYS_TIMER_INTERVAL_MODE;
	appTimer.handler = appTimerHandler;
}

/*************************************************************************//**
*****************************************************************************/
static void APP_TaskHandler(void)
{
	switch (appState) {
	case APP_STATE_INITIAL:
	{
		appInit();
		appState = APP_STATE_IDLE;
	}
	break;

	case APP_STATE_IDLE:
		break;

	default:
		break;
	}
	
	/* We can send a maximum of 109 bytes per packet. If each sample
	is 8 bits in length, we can send up to 109 samples per package
	thus we implement a buffer to store the 109 samples before
	transmitting the entire packet.*/
	adc_rx_length = adc_rx( adc_data, APP_RX_BUF_SIZE );
	if (adc_rx_length) {
		for (uint16_t i = 0; i < adc_rx_length; i++) {
			sio2host_putchar(adc_data[i]);
			if (appUartBufferPtr == sizeof(appUartBuffer)) {
				appSendData();
			}

			if (appUartBufferPtr < sizeof(appUartBuffer)) {
				appUartBuffer[appUartBufferPtr++] = adc_data[i];
			}
		}

		SYS_TimerStop(&appTimer);
		SYS_TimerStart(&appTimer);
	}
}

volatile uint8_t button_down;
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

uint8_t adc_rx(uint8_t *data, uint8_t max_length)
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

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	sysclk_init();
	board_init();
	/*Initiates the wireless system*/
	SYS_Init();
	/*Initiates internal ADC, select temp sensor to test 
	ADC functionality. By default, cont conversions*/
	adc_init( ADC_PRESCALER_DIV128, ADC_VREF_1V6, ADC_MUX_TEMP_SENSOR, true );
	cpu_irq_enable();

	while (1)
	{
		debounce();
		if ( button_down ){
			// Clear flag
			button_down = 0;
			adc_start_conversion();
		}
		SYS_TaskHandler();
		APP_TaskHandler();
	}

}
/* Receive the continuous conversion ADC samples, store them into buffer*/
ISR( ADC_vect ){
	uint8_t temp;
	/* Introducing critical section to avoid buffer corruption. */
	cpu_irq_disable();
	temp = ADCH;
	/* The number of data in the receive buffer is incremented and the
	 * buffer is updated. */
	adc_rx_buffer[adc_rx_buf_tail] = temp;

	if ((APP_RX_BUF_SIZE - 1) == adc_rx_buf_tail) {
		/* Reached the end of buffer, revert back to beginning of
		 * buffer. */
		adc_rx_buf_tail = 0x00;
	} else {
		adc_rx_buf_tail++;
	}

	cpu_irq_enable();

}

