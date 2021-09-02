/**
 * \file spi.c
 *
 * \author maczijewski, cfrauzem
 * \date Created: 21.09.2018 17:50:29, Modified 19.06.2019
 * 
 * \ingroup low_level_controller
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
// The communication is blocking / synchronous. 
// spi_exchange() blocks until the transfer is complete.

/**
 * \brief Index of byte which is to be transferred next.
 * \ingroup low_level_controller
 */
static volatile uint8_t spi_buffer_index = 0;

/**
 * \brief Buffer for data which will be sent to mid_level_controller.
 * \ingroup low_level_controller
 */
static volatile uint8_t* miso_buffer = 0;

/**
 * \brief Dependent on whether low_level_controller received the bytes of
 * 	      mid_level_controller correctly, its answer message will differ
 * 		  to indicate the success/failure to mid_level_controller. The data
 * 	      in this buffer here will be sent, if mosi-data is received correctly.
 * \ingroup low_level_controller
 */
static volatile uint8_t miso_correct_CRC[SPI_BUFFER_SIZE];

/**
 * \brief Dependent on whether low_level_controller received the bytes of
 * 	      mid_level_controller correctly, its answer message will differ
 * 		  to indicate the success/failure to mid_level_controller. The data
 * 	      in this buffer here will be sent, if mosi-data is NOT received correctly.
 * \ingroup low_level_controller
 */
static volatile uint8_t miso_wrong_CRC[SPI_BUFFER_SIZE];

/**
 * \brief Buffer for data which will be received from mid_level_controller.
 * \ingroup low_level_controller
 */
static volatile uint8_t mosi_buffer[SPI_BUFFER_SIZE];


/**
 * \brief interrupt for end of byte transfer
 * \ingroup low_level_controller
 */
ISR(SPI_STC_vect) {
	mosi_buffer[spi_buffer_index] = SPDR;
	spi_buffer_index++;
	SPDR = miso_buffer[spi_buffer_index];
}


/**
 * \brief interrupt for slave select pin change
 * \ingroup low_level_controller
 */
ISR(PCINT0_vect) {
	if(PINB & 1) { // end of spi transmission
		watchdog_reset(); // resume normal operation when the master raises the slave select
	}	
}



void spi_exchange(spi_miso_data_t *packet_send, spi_mosi_data_t *packet_received) 
{
	// calculate miso CRCs for two cases: mosi CRC ok, and mosi CRC wrong
	
	// good CRC
	{
		CLEAR_BIT(packet_send->status_flags, 1);
		packet_send->CRC = 0;
		packet_send->CRC = crcFast((uint8_t*)(packet_send), sizeof(spi_miso_data_t));
		uint8_t* p_send = (uint8_t*) packet_send;
		for (uint8_t i = 0; i < sizeof(spi_miso_data_t); i++) 
		{
			miso_correct_CRC[i] = p_send[i];
		}
	}
	
	// bad CRC
	{
		SET_BIT(packet_send->status_flags, 1);
		packet_send->CRC = 0;
		packet_send->CRC = crcFast((uint8_t*)(packet_send), sizeof(spi_miso_data_t));
		
		// copy stack data with CRC flag
		uint8_t* p_send = (uint8_t*) packet_send;
		for (uint8_t i = 0; i < sizeof(spi_miso_data_t); i++) 
		{
			miso_wrong_CRC[i] = p_send[i];
		}
	}
	
	// first send raspberry pi miso_wrong_CRC, since we have not received a correct packet yet
	miso_buffer = miso_wrong_CRC;
	
	// prepare transmission
	spi_buffer_index = 0;
	// set start byte for next transmission
	SPDR = miso_buffer[spi_buffer_index]; 
	
	// wait for transmission start
	// SS pin is PB1
	// HIGH == transmission idle
	// LOW == transmission start
	while(PINB & 0b00000001) 
	{
		if (safe_mode_flag) 
		{
			return;
		}
	}
	
	// spi transmission active
	while(1) 
	{
		if (spi_buffer_index >= SPI_BUFFER_SIZE) // data package transmission complete
		{
			// validate spi received mosi crc
			spi_mosi_data_t* mosi_buffer_as_mosi_packet = (spi_mosi_data_t*) mosi_buffer;
			const uint16_t mosi_CRC_actual = mosi_buffer_as_mosi_packet->CRC;
			mosi_buffer_as_mosi_packet->CRC = 0;
			const uint16_t mosi_CRC_target = crcFast(mosi_buffer, sizeof(spi_mosi_data_t));
			
			if (mosi_CRC_actual == mosi_CRC_target) 
			{
				// tell master that mosi message was received correctly
				miso_buffer = miso_correct_CRC;
				
				// copy mosi buffer data to output
				uint8_t* p_received = (uint8_t*) packet_received;
				for (uint8_t i = 0; i < sizeof(spi_mosi_data_t); i++) 
				{
					p_received[i] = mosi_buffer[i];
				}
			}
			
			// reset buffer index counter
			spi_buffer_index = 0;
			
			// set start byte for next transmission
			SPDR = miso_buffer[spi_buffer_index]; 
		}
	
		if (safe_mode_flag) 
		{
			return;
		}
		
		// end of transmission
		if (PINB & 0b00000001) 
		{
			return;
		}
	}
}


void spi_setup() 
{
	SET_BIT(SPCR, SPIE); // spi interrupt enable
	SET_BIT(SPCR, SPE); // spi enable
	// MSB first (default)
	// slave mode (default)
	// spi mode 0 (default)
	
	SET_BIT(DDRB, 3); // set MISO as output
	
	SET_BIT(PORTB, 1); // set SS as pull-up input
	CLEAR_BIT(MCUCR, PUD);
	
	
	// interrupt on the slave select pin
	SET_BIT(PCICR, PCIE0);
	SET_BIT(PCMSK0, PCINT0);
}