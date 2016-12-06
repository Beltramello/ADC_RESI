
//****************************************************//
//                                                    //
//                 ADC_Comparator                     //     
//                                                    // 
//****************************************************//

/*
Name:		ADC_Comparator
Created:	09/08/2016 08:17:16 AM
Author:		Carlos Beltramello
Company:	QL Labor - Continental AG
*/


#include <pwm_lib.h>
#include <pwm_defs.h>
#include <tc_lib.h>
#include <tc_defs.h>
#include"Arduino.h"
#include "variant.h"


//#define Serial SerialUSB

using namespace arduino_due::pwm_lib;

#define PWM_PERIOD_PIN_6 10000 // 10000 µsecs = 10ms 10kHz
#define PWM_DUTY_PIN_6 5000 // 1000 msecs in hundredth of usecs (1e-8 secs)


pwm<pwm_pin::PWML0_PC2> pwm_pin34;


int values = 0;
float voltage = 0;

int interval = 500;
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;

void ADC_Handler(void)
{
	uint32_t ul_mode;
	uint16_t us_adc;

	/* Disable Compare Interrupt. */
	adc_disable_interrupt(ADC, ADC_IDR_COMPE);

	if ((adc_get_status(ADC) & ADC_ISR_COMPE) == ADC_ISR_COMPE)
	{
		ul_mode = adc_get_comparison_mode(ADC);
		us_adc = adc_get_channel_value(ADC, ADC_CHANNEL_7);

		switch (ul_mode) {
		case 0:
			Serial.print("-ISR-:Potentiometer voltage   ");
			Serial.print(us_adc*(3.3 / 4096.0));
			Serial.print("[V]   is below the low threshold ->");
			Serial.println(1024 * (3.3 / 4096.0));
			break;

		case 1:
			Serial.print("-ISR-:Potentiometer voltage   ");
			Serial.print(us_adc*(3.3 / 4096.0));
			Serial.print("[V]   is above the high threshold ->");
			Serial.println(1024 * (3.3 / 4096.0));
			break;

		case 2:

			Serial.print("-ISR-:Potentiometer voltage   ");
			Serial.print(us_adc*(3.3 / 4096.0));
			Serial.print("[V]   is in the comparison window ->");
			Serial.print(1024 * (3.3 / 4096.0));
			Serial.print(" und ");
			Serial.println((3072 * (3.3 / 4096.0)));

			break;

		case 3:
			Serial.print("-ISR-:Potentiometer voltage   ");
			Serial.print(us_adc*(3.3 / 4096.0));
			Serial.print("[V]   is out of the comparison window ->");
			Serial.print(1024 * (3.3 / 4096.0));
			Serial.print(" und ");
			Serial.println((3072 * (3.3 / 4096.0)));
			break;

		default:
			Serial.print("-ISR-:Potentiometer voltage   ");
			Serial.print(us_adc*(3.3 / 4096.0));
			Serial.print("[V]   is out of the comparison window ->");
			Serial.print(1024 * (3.3 / 4096.0));
			Serial.print(" und ");
			Serial.println((3072 * (3.3 / 4096.0)));

		}
	}

}



void setup() {
	Serial.begin(9600);
	//ADC->ADC_MR |= 0x80;  //set free running mode on ADC
	//ADC->ADC_CHER = 0x80; //enable ADC on pin A0


	pmc_enable_periph_clk(ID_ADC);
	adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
	adc_disable_interrupt(ADC, 0xFFFFFFFF);
	adc_set_resolution(ADC, ADC_12_BITS);
	adc_configure_power_save(ADC, ADC_MR_SLEEP_NORMAL, ADC_MR_FWUP_OFF);
	adc_configure_timing(ADC, 1, ADC_SETTLING_TIME_3, 1);

	adc_configure_trigger(ADC, ADC_TRIG_SW, 1);

	//adc_configure_trigger(ADC, ADC_TRIG_PWM_EVENT_LINE_0, 0);

	adc_enable_channel(ADC, ADC_CHANNEL_7);
	//adc_enable_channel(ADC, ADC_CHANNEL_6);

	adc_set_comparison_channel(ADC, ADC_CHANNEL_7);

	adc_set_comparison_mode(ADC, ADC_EMR_CMPMODE_IN);

	adc_set_comparison_window(ADC, 1024, 3072);

	
	adc_set_bias_current(ADC, 1);
	adc_disable_tag(ADC);
	adc_disable_ts(ADC);
	adc_stop_sequencer(ADC);
	//adc_disable_channel_differential_input(ADC, ADC_CHANNEL_7);
	//adc_disable_all_channel(ADC);
	//adc_enable_channel(ADC, ADC_CHANNEL_7);
	//adc_configure_trigger(ADC, ADC_TRIG_SW, 1);
	NVIC_EnableIRQ(ADC_IRQn);
	adc_start(ADC);
	//pwm_pin34.start(PWM_PERIOD_PIN_6, PWM_DUTY_PIN_6);


	

	//pmc_enable_periph_clk(ID_ADC); // Enable the peripheral clock.

	//							   // Initialize the ADC.
	//adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
	//adc_disable_interrupt(ADC, 0xFFFFFFFF);
	//// Enable individual settings for the input channels.
	//adc_enable_anch(ADC);

	//adc_configure_timing(ADC, 2, ADC_SETTLING_TIME_3, 1); // Data transfer time.

	//adc_set_bias_current(ADC, 1);
	//// Configure the conversion resolution.
	//adc_set_resolution(ADC, ADC_12_BITS); // Use 12-bit resolution.
	//adc_configure_power_save(ADC, ADC_MR_SLEEP_NORMAL, ADC_MR_FWUP_OFF);
	//adc_configure_trigger(ADC, ADC_TRIG_SW, 1); // Free-running mode (no trigger is required for conversion).
	//adc_disable_all_channel(ADC);
	//// Enable the input channels.
	//adc_enable_channel(ADC, ADC_CHANNEL_2);
	//adc_enable_channel(ADC, ADC_CHANNEL_3);
	////adc_enable_channel(ADC, ADC_CHANNEL_0); // Pin A0.

	//// Configure channels as differential input.
	//adc_enable_channel_differential_input(ADC, ADC_CHANNEL_2);
	//adc_enable_channel_differential_input(ADC, ADC_CHANNEL_3);



	//adc_set_channel_input_gain(ADC, ADC_CHANNEL_2, ADC_GAINVALUE_0);
	//adc_set_channel_input_gain(ADC, ADC_CHANNEL_3, ADC_GAINVALUE_0);



	//// Start the ADC.
	//adc_start(ADC);

	uint32_t MR = ADC->ADC_MR;
	uint32_t CGR = ADC->ADC_CGR;
	Serial.println(CGR);
	Serial.println(MR);
}

void loop() {

	uint32_t currentMillis = millis();
	if (currentMillis - previousMillis >= interval)
	{

		while ((ADC->ADC_ISR & 0x80) == 0); // wait for conversion
	//while ((adc_get_status(ADC) & ADC_ISR_DRDY) != ADC_ISR_DRDY){}; //Wait for end of conversion

		values = ADC->ADC_CDR[7]; //get values
	  //values = adc_get_latest_value(ADC); // Read ADC
	  //Serial.println(values);
		//voltage = values *(3.3 / 4096.0);
		//Serial.println(voltage, 3);
		//Serial.print(values);
		//Serial.println(adc_get_comparison_mode(ADC));


		ADC_Handler();


		previousMillis = currentMillis;
	}




}

