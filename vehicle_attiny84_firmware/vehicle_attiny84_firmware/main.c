#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define SET_BIT(p,n) ((p) |= (1 << (n)))
#define CLEAR_BIT(p,n) ((p) &= ~((1) << (n)))



/****************************************************/
/*******************   Pinout  **********************/
/****************************************************/

// Package PDIP/SOIC

// 01 == VCC
// 02 == Oscillator
// 03 == Oscillator
// 04 == Programming
// 05 == Motor INA (Direction selector)
// 06 == Motor PWM
// 07 == Programming / SPI MOSI
// 08 == Programming / SPI MISO
// 09 == Programming / SPI SCK
// 10 == Motor INB (Direction selector)
// 11 == Servo PWM
// 12 == LED
// 13 == ADC battery voltage
// 14 == GND


/****************************************************/
/******************* Motor PWM **********************/
/****************************************************/


void motor_set_duty(uint16_t duty) // values from 0 to 200
{
	if(duty > 200) {
		duty = 200;
	}
	OCR0B = duty;
}

#define MOTOR_DIRECTION_BRAKE 0
#define MOTOR_DIRECTION_FORWARD 1
#define MOTOR_DIRECTION_REVERSE 2

void motor_set_direction(uint8_t direction)
{
	if(direction & 1) {
		SET_BIT(PORTA, 3);
	} else {
		CLEAR_BIT(PORTA, 3);
	}
	
	if(direction & 2) {
		SET_BIT(PORTB, 2);
	} else {
		CLEAR_BIT(PORTB, 2);
	}
}

void motor_pwm_setup()
{
	// PWM mode 5: Phase Correct
	SET_BIT(TCCR0A, WGM00);
	SET_BIT(TCCR0B, WGM02);
	
	// Set PWM frequency to 20kHz
	OCR0A = 200;
	
	// Enable output on Pin 6 / PORTA7 / OC0B
	SET_BIT(TCCR0A, COM0B1);
	SET_BIT(DDRA, 7);
	
	// no clock scaling, counter runs at 8MHz
	SET_BIT(TCCR0B, CS00);
	
	// Enable direction control
	SET_BIT(DDRB, 2);
	SET_BIT(DDRA, 3);
	motor_set_direction(MOTOR_DIRECTION_FORWARD);
}


/****************************************************/
/******************** Servo PWM *********************/
/****************************************************/


// Servo PWM rising edge interrupt
ISR(TIM1_OVF_vect)
{
	SET_BIT(PORTA, 2);
}

// Servo PWM falling edge interrupt
ISR(TIM1_COMPB_vect)
{
	CLEAR_BIT(PORTA, 2);
}

void servo_pwm_setup() 
{
	// PWM mode: phase & freq correct
	SET_BIT(TCCR1B, WGM13);
	SET_BIT(TCCR1A, WGM10);
	
	// Enable output
	SET_BIT(DDRA, 2);
	
	// Set frequency to 50Hz
	OCR1A = 10000;
	
	// Configure separate interrupts for rising and falling edge
	SET_BIT(TIFR1, OCF1B);
	SET_BIT(TIMSK1, OCIE1B);
	SET_BIT(TIFR1, TOV1);
	SET_BIT(TIMSK1, TOIE1);
	
	// Set /8 prescaler, results in 1 MHz
	SET_BIT(TCCR1B, CS11);
}

void servo_set_position(uint8_t val8) // from 0 to 250
{
	uint16_t val = 1000 + 4 * ((uint16_t)(val8));
	if(val > 2000) val = 2000;
	OCR1B = val;
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


int main(void)
{
	
	led_setup();
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
			_delay_us(300);
		}
		
	}
}