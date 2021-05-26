// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

/**
 * \file spi.h
 *
 * \date Created: 21.09.2018 19:42:56
 * \author maczijewski
 * 
 * \brief In order to make communication between mid_level_controller and
 *        low_level_controller possible the Serial Peripheral Interface (SPI)
 *        is used. This module here provides an interface to use SPI for
 *        communicating with the mid_level_controller.
 * 
 * \ingroup low_level_controller
 */ 


#ifndef SPI_H_
#define SPI_H_


#include "spi_packets.h"

/**
 * \brief Exchanges fixed bytes of data with the mid_level_controller via SPI.
 *        Note: This low_level_controller is the slave. Since the communication
 *              is synchronous, spi_exchange will block until the transfer is
 *              completed.
 * \param packet_send       The package to be sent to mid_level_controller.
 * \param packet_received   The received package from mid_level_controller.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void spi_exchange(spi_miso_data_t *packet_send, spi_mosi_data_t *packet_received);

/**
 * \brief Sets up all relevant registers such that SPI can be used.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
void spi_setup();


#endif /* SPI_H_ */