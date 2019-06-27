/*
 * watchdog.h
 *
 * Created: 6/21/2019 13:35:35
 *  Author: cfrauzem
 */ 


#ifndef WATCHDOG_H_
#define WATCHDOG_H_


#include "spi_packets.h"


//inline uint8_t get_safe_mode_flag(); // read only
extern volatile uint8_t safe_mode_flag; // read and write


void watchdog_disable();

void watchdog_enable();

void watchdog_reset();

void safe_mode(spi_mosi_data_t* spi_mosi_data);


#endif /* WATCHDOG_H_ */