/*
 * spi.c
 *
 * Created: 21.09.2018 17:50:29
 *  Author: maczijewski
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "util.h"
#include "spi.h"



// SPI protocol: exchange fixed size buffers (no register addressing)
// use double buffering to avoid data corruption
#define SPI_BUFFER_SIZE 32
volatile uint8_t spi_buffer_index = 0;

volatile uint8_t miso_buffer_a[SPI_BUFFER_SIZE];
volatile uint8_t miso_buffer_b[SPI_BUFFER_SIZE];
volatile uint8_t* miso_buffer_local = miso_buffer_a;
volatile uint8_t* miso_buffer_bus = miso_buffer_b;

volatile uint8_t mosi_buffer_a[SPI_BUFFER_SIZE];
volatile uint8_t mosi_buffer_b[SPI_BUFFER_SIZE];
volatile uint8_t* mosi_buffer_local = mosi_buffer_a;
volatile uint8_t* mosi_buffer_bus = mosi_buffer_b;

volatile uint8_t local_miso_buffer_has_new_data_flag = 0;

// interrupt for end of byte transfer
ISR(SPI_STC_vect) {
	uint8_t received_byte = SPDR;
	SPDR = miso_buffer_bus[spi_buffer_index]; // send next byte
	mosi_buffer_bus[spi_buffer_index] = received_byte;
	spi_buffer_index++;
}

// interrupt for slave select pin change
ISR(PCINT0_vect) {
	if(PINB & 1) { // end of spi transmission
		SPDR = 42; // set start byte for next transmission
		
		// swap received buffer
		volatile uint8_t* tmp = mosi_buffer_local;
		mosi_buffer_local = mosi_buffer_bus;
		mosi_buffer_bus = tmp;
	}
	else { // beginning of spi transmission
		spi_buffer_index = 0;
		
		// swap send buffer
		if(local_miso_buffer_has_new_data_flag) { // prevent resending old data
			volatile uint8_t* tmp = miso_buffer_local;
			miso_buffer_local = miso_buffer_bus;
			miso_buffer_bus = tmp;
			local_miso_buffer_has_new_data_flag = 0;
		}
	}
	
}

void spi_send(int32_t speed) {
	cli(); // stop buffer swap
	uint8_t* ptr = (uint8_t*)(&speed);
	miso_buffer_local[0] = ptr[0];
	miso_buffer_local[1] = ptr[1];
	miso_buffer_local[2] = ptr[2];
	miso_buffer_local[3] = ptr[3];
	
	local_miso_buffer_has_new_data_flag = 1;
	sei();
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