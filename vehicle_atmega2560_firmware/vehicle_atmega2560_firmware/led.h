/*
 * led.h
 *
 * Created: 20.09.2018 20:24:06
 *  Author: maczijewski
 */ 


#ifndef LED_H_
#define LED_H_


#include "spi_packets.h"


void led_set_state(spi_mosi_data_t* spi_mosi_data);
void toggle_led();
void led_setup();
void test_led(uint8_t led1,uint8_t led2,uint8_t led3,uint8_t led4);


#endif /* LED_H_ */