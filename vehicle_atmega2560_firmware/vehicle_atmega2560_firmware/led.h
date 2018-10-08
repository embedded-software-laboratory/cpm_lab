/*
 * led.h
 *
 * Created: 20.09.2018 20:24:06
 *  Author: maczijewski
 */ 


#ifndef LED_H_
#define LED_H_

#include "util.h"

static inline void led_set_state(uint32_t tick, uint8_t LED_bits) {
	uint8_t blink_fast_state = (tick >> 3) & 1;
	uint8_t blink_slow_state = (tick >> 5) & 1;
	
	uint8_t state_map[4] = {blink_fast_state, blink_slow_state, 0, 1};


	if(state_map[(LED_bits >> 0) & 0b00000011]) SET_BIT(PORTC, 0);
	else                                      CLEAR_BIT(PORTC, 0);
	
	if(state_map[(LED_bits >> 2) & 0b00000011]) SET_BIT(PORTC, 1);
	else                                      CLEAR_BIT(PORTC, 1);
	
	if(state_map[(LED_bits >> 4) & 0b00000011]) SET_BIT(PORTC, 2);
	else                                      CLEAR_BIT(PORTC, 2);
	
	if(state_map[(LED_bits >> 6) & 0b00000011]) SET_BIT(PORTC, 7);
	else                                      CLEAR_BIT(PORTC, 7);
}

static inline void led_setup() {	
	SET_BIT(DDRC, 0);
	SET_BIT(DDRC, 1);
	SET_BIT(DDRC, 2);
	SET_BIT(DDRC, 7);
}


#endif /* LED_H_ */