/*
 * led.h
 *
 * Created: 20.09.2018 20:24:06
 *  Author: maczijewski
 */ 


#ifndef LED_H_
#define LED_H_

#include "util.h"


#define ENABLE_RED_LED SET_BIT(PORTC, 0)
#define DISABLE_RED_LED CLEAR_BIT(PORTC, 0)
#define TOGGLE_RED_LED TOGGLE_BIT(PORTC, 0)

#define ENABLE_GREEN_LED SET_BIT(PORTC, 1)
#define DISABLE_GREEN_LED CLEAR_BIT(PORTC, 1)
#define TOGGLE_GREEN_LED TOGGLE_BIT(PORTC, 1)

#define ENABLE_BLUE_LED SET_BIT(PORTC, 2)
#define DISABLE_BLUE_LED CLEAR_BIT(PORTC, 2)
#define TOGGLE_BLUE_LED TOGGLE_BIT(PORTC, 2)



#endif /* LED_H_ */