/*
 * watchdog.h
 *
 * Created: 6/21/2019 13:35:35
 *  Author: cfrauzem
 */ 


#ifndef WATCHDOG_H_
#define WATCHDOG_H_


#include "spi_packets.h"

extern volatile uint8_t safe_mode_flag;

void watchdog_disable();

void watchdog_enable();

void watchdog_reset();

#endif /* WATCHDOG_H_ */