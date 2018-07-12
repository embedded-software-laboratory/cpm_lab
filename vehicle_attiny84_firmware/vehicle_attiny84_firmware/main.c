#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>


#define SET_BIT(p,n) ((p) |= (1 << (n)))
#define CLEAR_BIT(p,n) ((p) &= ~((1) << (n)))

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

uint16_t adc_read() {
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC) );
	uint16_t adc_val = ADC;
	return adc_val;
}

int main(void)
{
	
	SET_BIT(DDRA, 1); // enable LED	

	adc_setup();
	motor_pwm_setup();
	
	while (1)
	{
		PORTA ^= 0b10; // toggle LED
		
		uint16_t adc_val = adc_read();
		
		motor_set_duty(adc_val>>2);
		
		while(adc_val) {
			adc_val--;
			_delay_us(100);
		}
		
	}
}