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
#include "led.h"
#include "sysclk.h"
#include "phy.h"
#include "nwk.h"
#include "sysTimer.h"
#include "sio2host.h"
#include "spi.h"
#include "asf.h"

/*- Definitions ------------------------------------------------------------*/
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
#define		CONFIG0			0b11110010 // ext clock, no sensor bias, standby mode.
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

static uint8_t rx_data[100];

/*- Prototypes -------------------------------------------------------------*/
static void appSendData(void);

void adc_start_conversions(void);

uint8_t check_adc(void);

void config_ext_DR_irq(void);

/*Read from SPI buffer*/
uint8_t spi_rx(uint8_t *data, uint8_t max_length);

/*- Variables --------------------------------------------------------------*/
static SYS_Timer_t appTimer;
static NWK_DataReq_t appDataReq;
static bool appDataReqBusy = false;
static uint8_t appDataReqBuffer[100];
static uint8_t SpiBuffer[100];
static uint8_t	SpiBufferPtr = 0;
static uint8_t spi_rx_length;
uint8_t LADCs;
uint8_t HADCs;
bool button_down;
uint8_t pushes = 0;
uint8_t buffer_complete = 0;
uint8_t number_samples = 0;
uint8_t buff_count = 0;

/**
 * Receive buffer
 * The buffer size is defined in spi_random.h
 */
static uint8_t spi_rx_buf[2000];
//static uint8_t message[100];
/**
 * Receive buffer tail
 */
static uint8_t spi_rx_buf_tail = 0;

/**
 * Receive buffer head
 */
static uint8_t spi_rx_buf_head;
/**
 * Number of bytes in receive buffer
 */
static uint8_t spi_rx_count;


/*- Implementations --------------------------------------------------------*/

/****************************************************************************
*****************************************************************************/
static void appDataConf(NWK_DataReq_t *req)
{
	appDataReqBusy = false;
	(void)req;
}

/****************************************************************************
*****************************************************************************/
static void appSendData(void)
{
	if (appDataReqBusy || 0 == SpiBufferPtr ) {
		return;
	}
	
	//memcpy(appDataReqBuffer, SpiBuffer, SpiBufferPtr);
	
	appDataReq.dstAddr = 1 - APP_ADDR;
	appDataReq.dstEndpoint = APP_ENDPOINT;
	appDataReq.srcEndpoint = APP_ENDPOINT;
	appDataReq.options = 0;
	appDataReq.data = SpiBuffer;
	appDataReq.size = SpiBufferPtr;
	appDataReq.confirm = appDataConf;
	NWK_DataReq(&appDataReq);

	SpiBufferPtr = 0;
	//buffer_complete = 0;
	appDataReqBusy = true;
	LED_On(LED0);
}

/****************************************************************************
*****************************************************************************/

/*
static void appTimerHandler(SYS_Timer_t *timer)
{
	appSendData();
	(void)timer;
}*/

/****************************************************************************
*****************************************************************************/
static bool appDataInd(NWK_DataInd_t *ind)
{
	for (uint8_t i = 0; i < ind->size; i++) {
		sio2host_putchar(ind->data[i]);
	}
	//LED_Toggle(LED0);
	return true;
}

/****************************************************************************
*****************************************************************************/
static void appInit(void)
{
	NWK_SetAddr(APP_ADDR);
	NWK_SetPanId(APP_PANID);
	PHY_SetChannel(APP_CHANNEL);
	PHY_SetRxState(true);

	NWK_OpenEndpoint(APP_ENDPOINT, appDataInd);

	/*appTimer.interval = APP_FLUSH_TIMER_INTERVAL;
	appTimer.mode = SYS_TIMER_INTERVAL_MODE;
	appTimer.handler = appTimerHandler;*/
}

/****************************************************************************
*****************************************************************************/
static void APP_TaskHandler(void)
{
	spi_rx_length = spi_rx(rx_data, 100);
	if (spi_rx_length) {
		for (uint16_t i = 0; i < spi_rx_length; i++) 
		{
			if (SpiBufferPtr == sizeof(SpiBuffer))
			{
				appSendData();
			}
			if (SpiBufferPtr < sizeof(SpiBuffer)) 
			{
				SpiBuffer[SpiBufferPtr++] = rx_data[i];
			}
		}
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
	SPI_Transmit_no_IRQ(W_LOCK_REG); // inc write to lock the device.
	SPI_Transmit_no_IRQ(LOCK_on);
	PORTD |= (1<<SS2);// stop comms.*/
	/*If we lock the ADC configuration, when will we need to unlock it
	thet is, when will we need to reconfigure our ADC?*/
}
/****************************************************************************************
     ADC start conversions 
****************************************************************************************/
void adc_start_conversions(void)
{
	PORTD &= ~ ( 1 << SS2 );
	SPI_Transmit_no_IRQ(STRTCNV);
	PORTD |= ( 1 << SS2 );
} 
/**************************************************************************************
  Configures external irq on PIND0 
**************************************************************************************/
void config_ext_DR_irq(void)
{
	DDRD |= (0<<DDD0);//External interrupt on pin D0
	EICRA |= (0x00<<ISC00);// External interrupt 0 activates on falling edge
}
/****************************************************************************
  Check for correct configuration of the ADC
****************************************************************************/
uint8_t check_adc(void){
	
	PORTD &= ~ ( 1 << SS2 );
	uint8_t adc_ok = 0;
	uint8_t adc_config[5];
	SPI_Transmit_no_IRQ(R_ADC_CONFIG);
	adc_config[0] = SPI_Transmit_no_IRQ(DNT_CARE);
	adc_config[1] = SPI_Transmit_no_IRQ(DNT_CARE);
	adc_config[2] = SPI_Transmit_no_IRQ(DNT_CARE);
	adc_config[3] = SPI_Transmit_no_IRQ(DNT_CARE);
	adc_config[4] = SPI_Transmit_no_IRQ(DNT_CARE);
	PORTD |= (1<<SS2);
	for (int i = 0; i < 5; i++)
	{
		sio2host_putchar(adc_config[i]);
		switch( adc_config[i] )
		{
			case CONFIG0:
			case CONFIG1:
			case CONFIG2:
			case CONFIG3:
			case ADC_IRQ_POR:
			case ADC_IRQ_no:
				adc_ok++;
		}
		if( adc_ok == 5 )
		{
			adc_ok = 1;
		}
	}
	return adc_ok;
}
/**************************************************************************
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
uint8_t spi_rx(uint8_t *data, uint8_t max_length)
{
	uint8_t data_received = 0;
	buff_count++;
	
	if(buff_count == 20)
	{
		buff_count = 0;
	}
	
	
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
		*data = spi_rx_buf;
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
/*****************************************************************************
*****************************************************************************/
int main(void)
{
	irq_initialize_vectors();
	sysclk_init();
	board_init();
	SYS_Init();
	sio2host_init();
	cpu_irq_enable();
	//LED_On(LED0);
	appInit();
	init_SPI_mstr();
	config_ext_DR_irq();
	config_Ziggys_ADC();
	check_adc();
		
	while (1) {
		while(!(button_down))
		{
			//LED_On(LED0);
			debounce();
			if(button_down)
			{
				//button_down = 0;
				EIMSK |= (1<<INT0);//External interrupt 0 enabled
				adc_start_conversions();
			}
		}

		//LED_On(LED0);
	    cli();//External interrupt 0 enabled
		APP_TaskHandler();
		sei();
		SYS_TaskHandler();
		
	}
}
/*****************************************************************************
	External interrupt handler for external ADC data ready event
*****************************************************************************/
ISR(INT0_vect)
{
	number_samples++;
	//LED_On(LED0);
	/*Optimized code*/
	cpu_irq_disable();
	PORTD &= ~ ( 1 << SS2 );
	SPDR = R_ADC_SAMPLES;
	while(!(SPSR & (1<<SPIF))){};
	SPDR = DNT_CARE;
	while(!(SPSR & (1<<SPIF))){};
	spi_rx_buf[spi_rx_buf_tail++] = SPDR;//Read High byte from ADC
	SPDR = DNT_CARE;//the increment of the index buf_tail happens only after 
	while(!(SPSR & (1<<SPIF))){};//the evaluation of spi_rx.
	spi_rx_buf[spi_rx_buf_tail] = SPDR;//Read Low byte from ADC
	PORTD |= ( 1 << SS2 );
	
	if ((SPI_RX_BUF_SIZE_HOST - 1) == spi_rx_buf_tail) {
		/* Reached the end of buffer, revert back to beginning of
		 * buffer. */
		spi_rx_buf_tail = 0x00;
		
	}
	else
	{
		spi_rx_buf_tail++;
	}
	cpu_irq_enable();	
}