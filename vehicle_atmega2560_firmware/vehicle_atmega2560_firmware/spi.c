/*
 * spi.c
 *
 * Created: 21.09.2018 17:50:29
 *  Author: maczijewski
 * Modified 19.06.2019
 *  Author: cfrauzem
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include "util.h"
#include "servo_timer.h"
#include "watchdog.h"
#include "crc.h"
#include "spi.h"


// spi protocol: exchange fixed size buffers (no register addressing).
// this (atmega) is the spi slave.
// synchronous master slave communication. Single buffer enough.
static volatile uint8_t spi_buffer_index = 0;

static volatile uint8_t* miso_buffer;

static volatile uint8_t miso_correct_CRC[SPI_BUFFER_SIZE];

static volatile uint8_t miso_wrong_CRC[SPI_BUFFER_SIZE];

static volatile uint8_t mosi_buffer[SPI_BUFFER_SIZE];


// interrupt for end of byte transfer
ISR(SPI_STC_vect) {
	uint8_t received_byte = SPDR;
	SPDR = miso_buffer[spi_buffer_index]; // send next byte
	mosi_buffer[spi_buffer_index] = received_byte;
	spi_buffer_index++;
}


// interrupt for slave select pin change
ISR(PCINT0_vect) {
	if(PINB & 1) { // end of spi transmission
		watchdog_reset(); // resume normal operation when the master raises the slave select
	}	
}



void spi_exchange(spi_miso_data_t *packet_send, spi_mosi_data_t *packet_received) {
	// calculate CRC possibilities before so least time in between spi communication
	// good CRC
	CLEAR_BIT(packet_send->status_flags, 1);
	packet_send->CRC = 0;
	packet_send->CRC = crcFast((uint8_t*)(packet_send), sizeof(spi_miso_data_t));
	
	// copy stack data without CRC flag
	uint8_t* p_send = (uint8_t*) packet_send;
	for (uint8_t i = 0; i < sizeof(spi_miso_data_t); i++) {
		miso_correct_CRC[i] = p_send[i];
	}
	
	// bad CRC
	SET_BIT(packet_send->status_flags, 1);
	packet_send->CRC = 0;
	packet_send->CRC = crcFast((uint8_t*)(packet_send), sizeof(spi_miso_data_t));
		
	// copy stack data with CRC flag
	p_send = (uint8_t*) packet_send;
	for (uint8_t i = 0; i < sizeof(spi_miso_data_t); i++) {
		miso_wrong_CRC[i] = p_send[i];
	}
	
	// first send raspberry pi miso_wrong_CRC
	miso_buffer = miso_wrong_CRC;
	
	// prepare first byte
	spi_buffer_index = 0;
	SPDR = miso_buffer[spi_buffer_index]; // set start byte for next transmission
	spi_buffer_index++;
	
	// wait for transmission start
	// SS pIN: PB1
	// SS pin HIGH = transmission idle
	// SS pin LOW = transmission start
	while(PINB & 0b00000001) {
		// safe mode triggered
		if (safe_mode_flag) {
			//break; // break exits loop, return exits function
			return;
		}
	}
	
	// spi transmission active
	while(1) {
		// process data package
		if (spi_buffer_index == SPI_BUFFER_SIZE+1) { // data package transmission complete
			
			// validate spi received crc
			uint16_t mosi_CRC_actual = packet_received->CRC;
			packet_received->CRC = 0;
			uint16_t mosi_CRC_target = crcFast((uint8_t*)(packet_received), sizeof(spi_mosi_data_t));
			
			if (mosi_CRC_actual == mosi_CRC_target) {
				// tell raspberry pi message was received correctly
				miso_buffer = miso_correct_CRC;
				
				// write spi buffer data to stack
				uint8_t* p_received = (uint8_t*) packet_received;
				for (uint8_t i = 0; i < sizeof(spi_mosi_data_t); i++) {
					p_received[i] = mosi_buffer[i];
				}
			}
			else {
				// tell raspberry pi message was corrupted
				miso_buffer = miso_wrong_CRC;
			}
			
			// reset buffer index counter
			spi_buffer_index = 0;
		}
	
		// safe mode triggered
		if (safe_mode_flag) {
			return;
		}
		
		// end of transmission
		if (PINB & 0b00000001) {
			return;
		}
	}
}


void spi_setup() {
	SET_BIT(SPCR, SPIE); // spi interrupt enable
	SET_BIT(SPCR, SPE); // spi enable
	// MSB first (default)
	// slave mode (default)
	// spi mode 0 (default)
	
	SET_BIT(DDRB, 3); // set MISO as output
	
	// interrupt on the slave select pin
	SET_BIT(PCICR, PCIE0);
	SET_BIT(PCMSK0, PCINT0);
}