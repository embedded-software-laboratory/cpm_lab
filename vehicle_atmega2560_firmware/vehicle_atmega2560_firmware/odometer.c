/*
 * odometer.c
 *
 * Created: 20.09.2018 21:34:20
 *  Author: maczijewski
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "util.h"
#include "odometer.h"

static const int8_t direction_lookup[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

static volatile uint8_t hall_sensor_states_prev = 0;
static volatile uint16_t timer1_prev = 0;
static volatile uint8_t standstill_flag = 0;

static volatile int32_t odometer_count = 0;

#define ODOMETER_BUFFER_SIZE 4
static volatile uint16_t odometer_time_interval_buffer[ODOMETER_BUFFER_SIZE];
static volatile int8_t odometer_direction_buffer[ODOMETER_BUFFER_SIZE];
static volatile uint8_t odometer_buffer_index = 0;

// interrupt for hall sensor pin change
ISR(PCINT2_vect) {
	uint16_t timer1_now = TCNT1;
	uint8_t hall_sensor_states_now = PINK;
	
	// take the useful bits
	hall_sensor_states_now = 
	    ((hall_sensor_states_now >> 2) & 0b00000001) |
	    ((hall_sensor_states_now >> 3) & 0b00000010);
	
	// rotation direction is determined from the current and previous state
	uint8_t direction_indicator = (hall_sensor_states_now << 2) | hall_sensor_states_prev;
	int8_t direction = direction_lookup[direction_indicator];
	
	// update odometer
	odometer_count += direction;
	
	// store time interval and direction
	odometer_buffer_index = (odometer_buffer_index+1) % ODOMETER_BUFFER_SIZE;
	odometer_time_interval_buffer[odometer_buffer_index] = timer1_now - timer1_prev;
	odometer_direction_buffer[odometer_buffer_index] = direction;
	
	// Since we got an interrupt, we are moving
	standstill_flag = 0;
	
	hall_sensor_states_prev = hall_sensor_states_now;
	timer1_prev = timer1_now;
}

int16_t get_speed() {
	cli();
	uint16_t timer1_now = TCNT1;
	uint16_t time_since_last_interrupt = timer1_now - timer1_prev;
	if(time_since_last_interrupt > 1562) {
		// After 0.1 second, assume we are stopped.
		// Need to use a flag, because the timer overflows after 4 seconds.
		standstill_flag = 1;
	}
		
	if(standstill_flag) {
		sei();
		return 0;
	}
		
		
	// Calculate average speed		
	int32_t dt_sum = 0;
	int8_t ds_sum = 0;
	for (uint8_t i = 0; i < ODOMETER_BUFFER_SIZE; i++)
	{
		dt_sum += (int32_t)(odometer_time_interval_buffer[i]);
		ds_sum += odometer_direction_buffer[i];
	}
	sei();
			
	// The ds_sum is scaled to ensure that the lowest speed is well resolved
	// and that no integer overflows occur.
	// The units are arbitrary at this point. Physical units will be determined through calibration.
	int32_t ds_sum_scaled = ((int32_t)ds_sum) << 24;
	
	int32_t average_speed = ds_sum_scaled / dt_sum;
	int16_t average_speed_i16 = average_speed / 256;
	
	return average_speed_i16;		
}

int32_t get_odometer_count() { return odometer_count; }

void odometer_setup() {
	// Setup Timer1
	// Normal mode
	// pre-scaler 1024 => 15625 Hz
	// overflow after 4.19 sec
	// Timer value on TCNT1
	SET_BIT(TCCR1B, CS10);
	SET_BIT(TCCR1B, CS12);
	
	// Setup Pin Change Interrupt for PCINT 16-23, PORTK, Pins 89-82
	// Only using pins 87, 85 for now.
	SET_BIT(PCICR, PCIE2);
	SET_BIT(PCMSK2, PCINT18);
	SET_BIT(PCMSK2, PCINT20);
}
