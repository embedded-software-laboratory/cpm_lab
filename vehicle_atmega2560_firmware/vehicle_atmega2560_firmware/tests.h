/*
 * tests.h
 *
 * Created: 09/08/2019 09:48:25
 *  Author: Janis
 */ 


#ifndef TESTS_H_
#define TESTS_H_

#include "spi_packets.h"
#include <avr/io.h>

void tests_setup();

void tests_apply(uint32_t tick, spi_miso_data_t *packet_send, spi_mosi_data_t *packet_received);

#endif /* TESTS_H_ */