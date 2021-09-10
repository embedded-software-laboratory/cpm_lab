/**
 * \file led.c
 *
 * \author maczijewski
 * \date Created: 20.11.2018 09:50:00
 * 
 * \ingroup low_level_controller
 */ 


#include <avr/io.h>
#include <util/delay.h>
#include "led.h"
#include "util.h"
#include "watchdog.h"

/**
 * \brief This array determines how many ticks one ID-LED flashing period lasts. Together with
 * 		  \link identification_LED_enabled_ticks \endlink this specifies the pattern with which
 * 		  the ID-LED flashes dependend on the vehicle ID.
 * \ingroup low_level_controller
 */
uint8_t identification_LED_period_ticks[26]  = { 1, 4, 7, 10, 13, 16, 7, 10, 13, 16, 19, 10, 13, 16, 19, 22, 13, 16, 19, 22, 25, 16, 19, 22, 25, 28 };

/**
 * \brief This array determines for how many ticks in one period the ID-LED is on. Together with
 * 		  \link identification_LED_period_ticks \endlink this specifies the pattern with which
 * 		  the ID-LED flashes dependend on the vehicle ID.
 * 
 * \example Let * indicate that the ID-LED is on, while . indicates that it is off. Then we get
 * 			for vehicle 3 the following pattern running in a loop: **........
 * 
 * \ingroup low_level_controller
 */
uint8_t identification_LED_enabled_ticks[26] = { 0, 2, 2,  2,  2,  2, 5,  5,  5,  5,  5,  8,  8,  8,  8,  8, 11, 11, 11, 11, 11, 14, 14, 14, 14, 14 };

/**
 * \brief saves the ID assigned to the vehicle (received from mid_level_controller)
 * \ingroup low_level_controller
 */
uint8_t vehicle_id = 0;

void led_set_state(uint8_t vehicle_id_in) {
	// test mode 
	if ((PINA & 1) == 0) return; 
    
	// only update vehicle ID on successful transmissions
    if (vehicle_id_in != 0) {
        vehicle_id = vehicle_id_in;
    }

    if (!safe_mode_flag){
        SET_BIT(PORTC, 0);
        SET_BIT(PORTC, 1);
        SET_BIT(PORTC, 2);
    }
    else {
        CLEAR_BIT(PORTC, 0);
        CLEAR_BIT(PORTC, 1);
        CLEAR_BIT(PORTC, 2);

        if(get_tick() % 25 == 0)
        {
            SET_BIT(PORTC, 0);
            SET_BIT(PORTC, 1);
            SET_BIT(PORTC, 2);
        }
    }
}


void led_setup() {
	SET_BIT(DDRC, 0);
	SET_BIT(DDRC, 1);
	SET_BIT(DDRC, 2);
	SET_BIT(DDRC, 7);
}


void led_toggle()
{
	// test mode 
	if ((PINA & 1) == 0) return; 
	
	CLEAR_BIT(PORTC, 7);
	
	// safe mode
	if (safe_mode_flag && (get_tick() % 25 == 0)){
		SET_BIT(PORTC, 7);
	}
	// normal mode
	else if(get_tick() % identification_LED_period_ticks[vehicle_id] < identification_LED_enabled_ticks[vehicle_id])
    {
        SET_BIT(PORTC, 7);
    }
}

void led_test(uint8_t led1, uint8_t led2, uint8_t led3, uint8_t led4){
	CLEAR_BIT(PORTC, 0);
	CLEAR_BIT(PORTC, 1);
	CLEAR_BIT(PORTC, 2);
	CLEAR_BIT(PORTC, 7);
	
	if (led1) SET_BIT(PORTC, 0);
	if (led2) SET_BIT(PORTC, 1);
	if (led3) SET_BIT(PORTC, 2);
	if (led4) SET_BIT(PORTC, 7);
}
