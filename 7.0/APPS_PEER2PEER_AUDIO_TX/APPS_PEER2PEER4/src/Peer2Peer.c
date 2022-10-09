/**
 * \file Peer2Peer.c
 *
 * \brief Peer2Peer application implementation
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 *
 */

/**
 * \mainpage
 * \section preface Preface
 * This is the reference manual for the LWMesh Peer2Peer Application
 * The LightWeight Mesh Peer2Peer  implements a wireless UART application.Two nodes are used in this application
 * These two nodes must be configured with addresses 0x0001 and 0x0000 respectively.
 * To test this application,open a terminal for both the nodes.On entering text in the terminal the data is transmitted from one 
 * node to another node(0x0001 to 0x0000 and vice-versa)
 */
/*- Includes ---------------------------------------------------------------*/
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
#include "nwk.h"
#include "conf_sio2host.h"


/*- Definitions ------------------------------------------------------------*/
#ifdef NWK_ENABLE_SECURITY
  #define APP_BUFFER_SIZE     (NWK_MAX_PAYLOAD_SIZE - NWK_SECURITY_MIC_SIZE)
#else
  #define APP_BUFFER_SIZE     NWK_MAX_PAYLOAD_SIZE
#endif
#define		ADC_IDLE		0
#define		ADC_CONVERTING	1
/*ADC buffer size*/
#define		ADC_BUF_SIZE		109//Equals the NWK_MAX_PAYLOAD_SIZE

static uint8_t rx_data[APP_RX_BUF_SIZE];

/*- Types ------------------------------------------------------------------*/
typedef enum AppState_t {
	APP_STATE_INITIAL,
	APP_STATE_IDLE,
} AppState_t;

/*- Prototypes -------------------------------------------------------------*/
static void appSendData(void);

/*- Variables --------------------------------------------------------------*/
static AppState_t appState = APP_STATE_INITIAL;
static SYS_Timer_t appTimer;
static NWK_DataReq_t appDataReq;
static bool appDataReqBusy = false;
static uint8_t appDataReqBuffer[APP_BUFFER_SIZE];
static uint8_t appADCbuffer[APP_BUFFER_SIZE];
static uint8_t appUartBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBufferPtr = 0;
static uint8_t appADCbufferPtr = 0;
static uint8_t adc_length;
static uint8_t sio_rx_length;

uint32_t fs=0;//Sampling frequency
uint8_t adc_status = ADC_IDLE;
char cmnd1[9] = "transmit";
uint8_t rx_char = 0;
static uint8_t rx_cmnd[9];
uint8_t prev_char = 0;
uint8_t cnt = 0;
volatile uint8_t button_down;

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
	if (appDataReqBusy || 0 == appADCbufferPtr) {
		return;
	}

	memcpy(appDataReqBuffer, appADCbuffer, appADCbufferPtr);

	appDataReq.dstAddr = 1 - APP_ADDR;
	appDataReq.dstEndpoint = APP_ENDPOINT;
	appDataReq.srcEndpoint = APP_ENDPOINT;
	appDataReq.options = NWK_OPT_ENABLE_SECURITY;
	appDataReq.data = appDataReqBuffer;
	appDataReq.size = appADCbufferPtr;
	appDataReq.confirm = appDataConf;
	NWK_DataReq(&appDataReq);

	appADCbufferPtr = 0;
	appDataReqBusy = true;
	LED_Toggle(LED0);
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
#ifdef PHY_AT86RF212
	PHY_SetBand(APP_BAND);
	PHY_SetModulation(APP_MODULATION);
#endif
	PHY_SetRxState(true);

	NWK_OpenEndpoint(APP_ENDPOINT, appDataInd);

	appTimer.interval = APP_FLUSH_TIMER_INTERVAL;
	appTimer.mode = SYS_TIMER_INTERVAL_MODE;
	appTimer.handler = appTimerHandler;
}
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
/*Function which waits till order to start conversion is received by UART*/
static void check_order_and_start_convertions(void){
	while(adc_status == ADC_IDLE){
		rx_char = usart_getchar( USART_HOST );
		if( rx_char == cmnd1[cnt] ){//Analyze if received instruction
			rx_cmnd[cnt] = rx_char;//matches any of the defined ones
			usart_putchar(USART_HOST, rx_char);
			cnt++;
		}
		if( cnt == 8 ){
			if( strcmp( (char *) rx_cmnd, cmnd1 ) == 0 ){
				memset(rx_cmnd, 0, sizeof(rx_cmnd));
				cnt = 0 ;
				adc_status = ADC_CONVERTING;
				adc_start_conversion();
			}
		}
	}
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
	//check_order_and_start_convertions();
	debounce();
	if( button_down ){
		button_down = 0;
		adc_start_conversion();
		adc_length = read_adc_buff(rx_data, APP_RX_BUF_SIZE);
	}
	if (adc_length) {
		for (uint16_t i = 0; i < adc_length; i++) {
			sio2host_putchar(rx_data[i]);
			if (appADCbufferPtr == sizeof(appADCbuffer)) {
				appSendData();
			}

			if (appADCbufferPtr < sizeof(appADCbuffer)) {
				appADCbuffer[appADCbufferPtr++] = rx_data[i];
			}
		}

		SYS_TimerStop(&appTimer);
		SYS_TimerStart(&appTimer);
	}
}

/*
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
	sio_rx_length = sio2host_rx(rx_data, APP_RX_BUF_SIZE);
	if (sio_rx_length) {
		for (uint16_t i = 0; i < sio_rx_length; i++) {
			sio2host_putchar(rx_data[i]);
			if (appUartBufferPtr == sizeof(appUartBuffer)) {
				appSendData();
			}

			if (appUartBufferPtr < sizeof(appUartBuffer)) {
				appUartBuffer[appUartBufferPtr++] = rx_data[i];
			}
		}

		SYS_TimerStop(&appTimer);
		SYS_TimerStart(&appTimer);
	}
}*/

/*************************************************************************//**
*****************************************************************************/
int main(void)
{
	irq_initialize_vectors();
	sysclk_init();
	board_init();
	SYS_Init();
	sio2host_init();
	fs = adc_init( ADC_PRESCALER_DIV16, ADC_VREF_1V6, ADC_MUX_ADC0, true, true, ADC_AUTOTRIGGER_SOURCE_FREERUNNING );
	cpu_irq_enable();
	usart_tx_complete_interrupt_disable(USART_HOST);
	usart_rx_complete_interrupt_disable(USART_HOST);
	usart_data_empty_interrupt_disable(USART_HOST);
	//LED_On(LED0);
	while (1) {
		SYS_TaskHandler();
		APP_TaskHandler();
	}
}
