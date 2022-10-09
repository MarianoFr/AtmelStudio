/*
 * adc.c
 *
 * Created: 26/11/2019 16:32:10
 *  Author: MarianoF
 */ 
/**
 * \brief ADC initialization. Set ADC prescaler and enable the ADC
 *
 * \param prescaler   ADC clock prescaler
 */
#include "adc.h"

static inline uint32_t adc_init(enum adc_prescaler prescaler, enum adc_voltage_reference vref, 
								enum adc_mux_setting input, bool irq_e, bool adate,
								enum adc_auto_trigger_source trg )
{
	uint32_t sampling_rate=0;
	uint8_t pres=0;
	
	sysclk_enable_module( POWER_RED_REG0,  PRADC_bm );
	ADCSRA = (uint8_t)prescaler | (1 << ADEN);
	while( ! (ADCSRB & (1 << AVDDOK) ));
	ADCSRC = 10<<ADSUT0; // set start-up time
	if(irq_e){
		ADCSRA |= (1 << ADIE); //Enable ADC interrupt
	}
	if(adate){
		adc_enable_autotrigger();
		adc_set_autotrigger_source(trg);
	}
	adc_set_voltage_reference( vref ); //set voltage ref
	adc_set_mux( input );
	ADCSRC |=  (1 << ADTHT1 | 1 << ADTHT0);//Set conversion time
	uint8_t adtht = (((ADCSRC) & (ADC_THT_MASK))>>ADTHT0);
	ADMUX |= (1 << ADLAR);
	if( (adate==true) &&  trg == 0 ){
			if(prescaler == 0){
			 pres = 2;
		}
		else{
			 pres = pow((double) 2,(double) prescaler);
		}
		if( input <= 7 ){
			sampling_rate = sysclk_get_main_hz()/((adtht +12)*pres);
		}
		else if( 7 < input && input < 0x1D ){
			sampling_rate = sysclk_get_main_hz()/((2*adtht +14)*pres);
		}
	}
	return sampling_rate;
}

