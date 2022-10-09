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
/* To the PEER2PEER application we add the SPI external ADC samples transmition*/ 
/*- Includes ---------------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "sys.h"
#include "led.h"
#include "sysclk.h"
#include "phy.h"
#include "nwk.h"
#include "sysTimer.h"
#include "sio2host.h"
#include "asf.h"
#include "spi_random.h"
#include "conf_sio2host.h"
/*- Definitions for LWMesh--------------------------------------------------*/
#ifdef NWK_ENABLE_SECURITY
  #define APP_BUFFER_SIZE     (NWK_MAX_PAYLOAD_SIZE - NWK_SECURITY_MIC_SIZE)
#else
  #define APP_BUFFER_SIZE     NWK_MAX_PAYLOAD_SIZE
#endif

/*- Definitions for external ADC configuration------------------------------*/

#define		ADC_IRQ_pin		CONFIG_EXT_INT7/*External interrupt pin which will announce the
											 MCU of a new data ready event*/
/*Initial configuration for the ADC*/

#define		START_CONFIG	0b01000110 // cmmnd byte 0, divc addss 01, reg add 0x1, inc write.
#define		W_LOCK_REG		0b01110110 // cmmnd byte 1, divc addss 01, reg add 0xD, inc write to unlock mem.
#define		DNT_CARE		0b01000000 // don't care cmmnd.
#define		R_ADC_SAMPLES	0b01000001 // ADCDATA read cmmnd.
#define		R_ADC_CONFIG	0b01000111 // Incremental read to check correct configuration of device.
#define		STRTCNV			0b01101000 // Start conversion fast command.
#define		STNDBYMOD		0b01101100 // Puts ADC on STANDBY mode fast command.
#define		CONFIG0			0b11110000 // ext clock, no sensor bias, standby mode.
#define		CONFIG1			0b00000000 // clk pres 1, OSR 32.
#define		CONFIG2			0b10001011 // BOOST=1, GAIN=1, AZ_MUX=0.
#define		CONFIG3			0b11000000 // cont conv, default crc comms check, crc comms disabled, offcal and gain cal disabled.
#define		ADC_IRQ			0b00000110 // IRQ Status Reg, irq mod all irq on pin, inact stt logic 1, fst cmmd enabled, conversion start disabled.
#define		ADC_IRQ_POR		0b01100110 // IRQ Status Reg, mask to check IRQ states is POR irq?.
#define		ADC_IRQ_no		0b01110110 // IRQ Status Reg, mask to check IRQ states is no irq?.
#define		MUX				0b00000001 // wont be used for now, its default value matches our need.
#define		SCAN			0x0000     // wont be written, 2 bytes long.
#define		OFFSETCAL		0x00	   // wont be written till deemed necessary.
#define		TIMER			0x000000   //
#define		GAINCAL			0x00	   // wont be written till deemed necessary
#define		LOCK_on			0x00	   // writing any value but the correct one to this register, locks the register map of the ADC
#define		LOCK_off		0xA5	   // correct key to unlock memory map of ADC	
#define		ADC_STAT_DR		0b00010011 // ADC Status Byte DR Mask
#define		ADC_STAT_CRC	0b00010101 // ADC Status Byte CRC checksum on the register map error Mask
#define		ADC_STAT_POR	0b00010110 // ADC Status Byte POR Mask

static uint8_t rx_data[APP_RX_BUF_SIZE];//max payload

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
static uint8_t appUartBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBufferPtr = 0;
static uint8_t sio_rx_length;
static uint8_t spi_rx_length;
static uint8_t number_of_samples = 0;
uint8_t LADCs;
uint8_t HADCs;
uint8_t MADCs;
uint8_t STATUS_byte = 0;
bool button_down;
uint8_t pushes = 0;
bool sample_ready = false;
// Counter for number of equal states
static uint8_t count = 0;
// Keeps track of current (debounced) state
static uint8_t button_state = 0;



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
	LED_Toggle(LED0);
}
/***************************************************************************************/
/* Flushes the buffer if 20ms have passed and no completion of the buffer has happened */
/***************************************************************************************/
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
/*
#ifdef PHY_AT86RF212
	PHY_SetBand(APP_BAND);
	PHY_SetModulation(APP_MODULATION);
#endif*/
	PHY_SetRxState(true);

	NWK_OpenEndpoint(APP_ENDPOINT, appDataInd);
	// These are the initialization statements in the appInit function.
	appTimer.interval = APP_FLUSH_TIMER_INTERVAL;
	appTimer.mode = SYS_TIMER_INTERVAL_MODE;
	 // TimerHandler will be called once after interval milliseconds.
	 // When (in HAL_UartBytesReceived()) are called the SYS_TimerStop()
	 // and then SYS_TimerStart(), timer starts again so appTimerHandler() function
	 // is performed every 20 ms.
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
	sio_rx_length = sio2host_rx( rx_data, APP_RX_BUF_SIZE );
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
}
/*************************************************************************//**
*****************************************************************************/
/* Write ADC registers via SPI */
void config_Ziggys_ADC( void ){
	PORTD &= ~ ( 1 << SS2 );// Put SS LOW to start transmission.
	SPI_Transmit_no_IRQ(START_CONFIG);
	SPI_Transmit_no_IRQ(CONFIG0);
	SPI_Transmit_no_IRQ(CONFIG1);
	SPI_Transmit_no_IRQ(CONFIG2);
	SPI_Transmit_no_IRQ(CONFIG3);
	SPI_Transmit_no_IRQ(ADC_IRQ);// Here we end the inc write of the ADC.
	PORTD |= (1<<SS2);	  // Put SS HIGH to stop comm.
	
	/* The following lines lock the ADC memory */
	
	/*PORTD &= ~ ( 1 << SS2 );// Restart comms as to write the pending registers.
	SPI_Transmit_no_IRQ(CMMND1); // inc write to lock the device.
	SPI_Transmit_no_IRQ(LOCK_on);
	PORTD |= (1<<SS2);// stop comms.*/
}
/***************************************************************************************/
/* ADC start conversions */
/***************************************************************************************/
void adc_start_conversions(void)
{
	SPCR &= ~ (1<<SPIE);//disable SPI interrupt
	PORTD &= ~ ( 1 << SS2 );
	SPI_Transmit_no_IRQ(STRTCNV);
	PORTD |= ( 1 << SS2 );
} 
/***************************************************************************************/
/* Configures external irq on PIND0 */
/***************************************************************************************/
void config_ext_DR_irq(void)
{
	DDRD |= (0<<DDD0);//External interrupt on pin D0
	EICRA |= (0x00<<ISC00);// External interrupt 0 actives on falling edge
}
/*************************************************************************//**
/*Check for correct configuration of the ADC*/
/*****************************************************************************/
uint8_t check_adc(void){
	
	PORTD &= ~ ( 1 << SS2 );
	uint8_t adc_ok = 0;
	SPI_Transmit_no_IRQ(R_ADC_CONFIG);
	SPCR |= (1<<SPIE);//enable SPI interrupt
	SPI_Transmit_IRQ(DNT_CARE);
	SPI_Transmit_IRQ(DNT_CARE);
	SPI_Transmit_IRQ(DNT_CARE);
	SPI_Transmit_IRQ(DNT_CARE);
	SPI_Transmit_IRQ(DNT_CARE);
	PORTD |= (1<<SS2);
	spi_rx_length = spi_rx( rx_data, APP_RX_BUF_SIZE );
	if(spi_rx_length)
	{
		for (int i = 0; i < spi_rx_length; i++)
		{
			sio2host_putchar(rx_data[i]);
			switch( rx_data[i] )
			{
				case CONFIG0:
				case CONFIG1:
				case CONFIG2:
				//case CONFIG3:
				case ADC_IRQ_POR:
				case ADC_IRQ_no:
					adc_ok++;
			}
			if( adc_ok == 5 )
			{
				adc_ok = 1;
			}
		}
	}
	return adc_ok;
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
				button_down = true;
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
int main(void)
{
	irq_initialize_vectors();
	sysclk_init();
	board_init();
	//SYS_Init();
	cpu_irq_enable();
	sio2host_init();
	init_SPI_mstr();
	uint8_t configuring[] = "Configuring ADC";
	config_ext_DR_irq();
	usart_serial_write_packet(USART_HOST,configuring, 15);
	config_Ziggys_ADC();
	check_adc();
	sample_ready = false;
	while (1) {			
		
		/*SYS_TaskHandler();
		APP_TaskHandler();*/
		/*if(number_of_samples>=100)
		{
			PORTD &= ~ ( 1 << SS2 );
			SPDR = STNDBYMOD;
			while(!(SPSR & (1<<SPIF))){};
			SPDR = 0;
			PORTD |= ( 1 << SS2 );
			number_of_samples = 0;
		}*/
		/*if(sample_ready==true)
		{
			sample_ready = 0;	
			while(!UCSR1A & USART_DRE_bm);
			UDR1 = HADCs;
			while(!UCSR1A & USART_DRE_bm);
			UDR1 = MADCs;
			while(!UCSR1A & USART_DRE_bm);
			UDR1 = LADCs;
		}*/
		while(!button_down)
		{
			debounce();
			if(button_down)
			{
				EIMSK |= (1<<INT0);//External interrupt 0 enabled
				adc_start_conversions();
			}
		}
		
		/*if( button_down )
		{
			//clear button
			button_down = 0;
			pushes++;	
			switch(pushes)
			{
				case 1:
					usart_serial_write_packet(USART_HOST,configuring, 15);
					config_Ziggys_ADC();
					break;
				case 2:
					check_adc();
					break;
				case 3:
					EIMSK |= (1<<INT0);//External interrupt 0 enabled
					adc_start_conversions();
					break; 
				default:
					break;	
			}
		}*/
	}
}
		
/*************************************************************************//**
	External interrupt handler for external ADC data ready event
*****************************************************************************/
ISR(INT0_vect)
{
	/*Optimized code*/
	cpu_irq_disable();
	/*Send don't care command to ADC to retrieve STATUS BYTE */
	/*PORTD &= ~ ( 1 << SS2 ); // Put Slave Select low to start transmission
	SPDR = DNT_CARE;
	while(!(SPSR & (1<<SPIF))){};// SDO pin in the slave transmits its STATUS BYTE.
	STATUS_byte = SPDR;
	PORTD |= ( 1 << SS2 );
	if ( STATUS_byte == ADC_STAT_DR )         
	{*/
		
		PORTD &= ~ ( 1 << SS2 );
		SPDR = R_ADC_SAMPLES;
		while(!(SPSR & (1<<SPIF))){};
		//STATUS_byte = SPDR; Consejo de Lenon, ya sabes a quien putear
		SPDR = DNT_CARE;
		while(!(SPSR & (1<<SPIF))){};
		HADCs = SPDR;
		SPDR = DNT_CARE;
		while(!(SPSR & (1<<SPIF))){};
		MADCs = SPDR;
		/*SPDR = DNT_CARE;
		while(!(SPSR & (1<<SPIF))){};
		LADCs = SPDR;*/
		PORTD |= ( 1 << SS2 );
		while(!UCSR1A & USART_DRE_bm);
		UDR1 = HADCs;
		while(!UCSR1A & USART_DRE_bm);
		UDR1 = MADCs;
		/*while(!UCSR1A & USART_DRE_bm);
		UDR1 = LADCs;*/
		//number_of_samples++;
	//}
	
	cpu_irq_enable();	
}