#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "usiTwiSlave.h"

#define SET_BIT(p,n) ((p) |= (1 << (n)))
#define CLEAR_BIT(p,n) ((p) &= ~((1) << (n)))



/****************************************************/
/******************* Motor PWM **********************/
/****************************************************/

void motor_pwm_setup() 
{
	// PWM mode 9: Phase & Freq. Correct
	SET_BIT(TCCR1A, WGM10);
	SET_BIT(TCCR1B, WGM13);
		
	// Target frequency: 20kHz PWM, calculation: 8 MHz / 20kHz / 2 == 200
	OCR1A = 200;
		
	// Enable output
	SET_BIT(TCCR1A, COM1B1);
	SET_BIT(DDRA, 5);
		
	// no clock scaling, counter runs at 8MHz
	SET_BIT(TCCR1B, CS10);
}

void motor_set_duty(uint16_t duty) // values from 0 to 200
{
	OCR1B = duty;
}


/****************************************************/
/******************** Servo PWM *********************/
/****************************************************/


// Servo PWM rising edge interrupt
uint8_t servo_pwm_step_mask = 0;
ISR(TIM0_OVF_vect)
{
	if(servo_pwm_step_mask >= 19) {
		SET_BIT(PORTA, 2);
		servo_pwm_step_mask = 0;
	} else {
		servo_pwm_step_mask++;
	}	
}

// Servo PWM falling edge interrupt
ISR(TIM0_COMPB_vect)
{
	if(servo_pwm_step_mask > 0) {
		CLEAR_BIT(PORTA, 2);
	}
}

void servo_pwm_setup() 
{
	// PWM mode: fast PWM
	SET_BIT(TCCR0A, WGM00);
	SET_BIT(TCCR0A, WGM01);	
	SET_BIT(TCCR0B, WGM02);
	
	// Enable output
	SET_BIT(DDRA, 2);
	
	// Reset counter after 124 steps. This results in a reset rate of 1kHz.
	OCR0A = 124;
	
	// Configure separate interrupts for rising and falling edge
	SET_BIT(TIFR0, OCF0B);
	SET_BIT(TIMSK0, OCIE0B);
	SET_BIT(TIFR0, TOV0);
	SET_BIT(TIMSK0, TOIE0);
	
	// Set /64 prescaler, results in 125 kHz
	SET_BIT(TCCR0B, CS01);
	SET_BIT(TCCR0B, CS00);
}

void servo_set_position(uint8_t val) // from 0 to 125
{
	if(val > 124) val = 124;
	OCR0B = val;
}

/****************************************************/
/*********************** ADC ************************/
/****************************************************/

void adc_setup()
{
	//// ADC setup
	// Use ADC on Pin 13 == PA0 == ADC0
	// Use V_CC as voltage reference
	ADMUX = 0;
	SET_BIT(ADCSRA, ADEN); // Enable ADC
	SET_BIT(ADCSRA, ADPS2); // ADC prescaler = 128
	SET_BIT(ADCSRA, ADPS1); // ADC prescaler = 128
	SET_BIT(ADCSRA, ADPS0); // ADC prescaler = 128
}

void adc_read() {
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC) );
}


/****************************************************/
/*********************** LED ************************/
/****************************************************/


void led_setup()
{
	SET_BIT(DDRA, 1);
}

void led_set(uint8_t status)
{
	if(status)
	{
		SET_BIT(PORTA, 1);
	}
	else
	{
		CLEAR_BIT(PORTA, 1);
	}
}

/****************************************************/
/*********************** I2C ************************/
/****************************************************/

#define I2C_SLAVE_ADDR 0x26

void i2c_slave_receive_event(uint8_t num_bytes)
{
	if(num_bytes == 3) // expected message size
	{
		led_set(usiTwiReceiveByte());
		motor_set_duty(usiTwiReceiveByte());
		servo_set_position(usiTwiReceiveByte());
	}
	else // invalid message, flush buffer
	{
		while(num_bytes) {
			usiTwiReceiveByte();
			num_bytes--;
		}
	}
}

void i2c_slave_request_event()
{
	usiTwiTransmitByte(ADCH);
	usiTwiTransmitByte(ADCL);
}

void i2c_slave_setup()
{
	usiTwiSlaveInit(I2C_SLAVE_ADDR);
	usi_onReceiverPtr = i2c_slave_receive_event;
	usi_onRequestPtr = i2c_slave_request_event;
	// TODO maybe disable watch dog timer !?
}



int main(void)
{
	
	led_setup();
	i2c_slave_setup();
	adc_setup();
	motor_pwm_setup();
	servo_pwm_setup();
	
	
	sei();
	
	while (1)
	{
		PORTA ^= 0b10; // toggle LED
		
		adc_read();
		
		motor_set_duty(ADC>>2);
		servo_set_position(ADC>>2);
		
		uint16_t adc_val = ADC;
		while(adc_val) {
			adc_val--;
			_delay_us(100);
		}
		
	}
}