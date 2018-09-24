/*
 * spi.h
 *
 * Created: 21.09.2018 19:42:56
 *  Author: maczijewski
 */ 


#ifndef SPI_H_
#define SPI_H_


#include "spi_packets.h"

void spi_send(spi_miso_data_t *packet);
void spi_receive(spi_mosi_data_t *packet);

void spi_setup();



#endif /* SPI_H_ */