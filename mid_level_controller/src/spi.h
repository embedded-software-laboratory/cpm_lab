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

#pragma once
#include "../../low_level_controller/vehicle_atmega2560_firmware/spi_packets.h"

/**
 * \brief Sets up all relevant registers such that SPI can be used.
 * \ingroup vehicle
 */
void spi_init();

/**
 * \brief Exchanges fixed bytes of data with the mid_level_controller via SPI.
 *        Note: This mid_level_controller is the master.
 * \ingroup vehicle
 * 
 * \param spi_mosi_data The package to be sent to low_level_controller.
 * \param spi_miso_data_out After transfer is finished, this variable contains the received
 *                          package from low_level_controller.
 * \param n_transmission_attempts_out After transfer is finished, this variable indicates the
 *                                    number of transmission attempts which were needed.
 * \param transmission_successful_out After transfer is finished, this flag indicates whether
 *                                    the transmission was successful (1) or not (0).
 */
void spi_transfer(
    spi_mosi_data_t spi_mosi_data,
    spi_miso_data_t *spi_miso_data_out,
    int *n_transmission_attempts_out,
    int *transmission_successful_out
);