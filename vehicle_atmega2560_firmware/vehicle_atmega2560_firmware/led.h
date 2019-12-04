/*
 * led.h
 *
 * Created: 20.09.2018 20:24:06
 *  Author: maczijewski
 */ 


#ifndef LED_H_
#define LED_H_


#include "spi_packets.h"


uint8_t[26] identification_LED_period_ticks  { 1, 4, 7, 10, 13, 16, 7, 10, 13, 16, 19, 10, 13, 16, 19, 22, 13, 16, 19, 22, 25, 16, 19, 22, 25, 28 };
uint8_t[26] identification_LED_enabled_ticks { 0, 2, 2,  2,  2,  2, 5,  5,  5,  5,  5,  8,  8,  8,  8,  8, 11, 11, 11, 11, 11, 14, 14, 14, 14, 14 };
uint8_t vehicle_id = 1;
uint32_t tick_count = 0;

void led_set_state(spi_mosi_data_t* spi_mosi_data);

void led_setup();


#endif /* LED_H_ */