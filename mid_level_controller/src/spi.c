#include <bcm2835.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "spi.h"
#include "../../low_level_controller/vehicle_atmega2560_firmware/crc.h"

/**
 * \file spi.c
 * \ingroup vehicle
 */

/**
 * \brief Checks whether the crc value provided by the low_level_controller is correct.
 * \param spi_miso_data Data, which has been sent from low_level_controller.
 * \ingroup vehicle
 */
static bool check_CRC_miso(spi_miso_data_t spi_miso_data) { 
    uint16_t mosi_CRC = spi_miso_data.CRC;
    spi_miso_data.CRC = 0;
    return mosi_CRC == crcFast((uint8_t*)(&spi_miso_data), sizeof(spi_miso_data_t));
}

/**
 * \brief The purpose of this variable is just to make sure that for-loop in busy wait
 *        is not optimized away.
 * \ingroup vehicle
 */
static volatile int dummy = 0;

/**
 * \brief Busy Wait in terms of for-loop cycles.
 * \param n The number of for-loop cycles which should be done to wait.
 * \ingroup vehicle
 */
static void busy_wait(int n) {
    for (int i = 0; i < n; ++i)
    {
        dummy++;
    }
}

void spi_init() {
    if (!bcm2835_spi_begin()) {
        printf("bcm2835_spi_begin failed. Are you running as root??\n");
        exit(EXIT_FAILURE);
    }

    crcInit();

    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_256);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);

    // enable CS pin
    bcm2835_gpio_fsel(RPI_GPIO_P1_24, BCM2835_GPIO_FSEL_OUTP);
    usleep(1000);

    // Default CS to high
    bcm2835_gpio_set(RPI_GPIO_P1_24);
    usleep(1000);
}


void spi_transfer(
    spi_mosi_data_t spi_mosi_data,
    spi_miso_data_t *spi_miso_data_out,
    int *n_transmission_attempts_out,
    int *transmission_successful_out
)
{
    *transmission_successful_out = 0;

    spi_mosi_data.CRC = crcFast((uint8_t*)(&spi_mosi_data), sizeof(spi_mosi_data_t));

    // CS low => transmission start
    bcm2835_gpio_clr(RPI_GPIO_P1_24);

    for (int i = 1; i < 4; ++i) // Try transmission 3 times at most
    {
        uint8_t SPI_recv_buffer[SPI_BUFFER_SIZE];
        uint8_t* mosi_data_ptr = (uint8_t*)(&spi_mosi_data);


        busy_wait(2000);

        for (int i = 0; i < SPI_BUFFER_SIZE; ++i)
        {
            SPI_recv_buffer[i] = bcm2835_spi_transfer(mosi_data_ptr[i]);
            busy_wait(3000);
        }

        busy_wait(10000);

        *n_transmission_attempts_out = i;

        memcpy(spi_miso_data_out, SPI_recv_buffer, sizeof(spi_miso_data_t));

        if(check_CRC_miso(*spi_miso_data_out) && !(spi_miso_data_out->status_flags & 2))
        {
            *transmission_successful_out = 1;
            break;
        }
    }

    // CS high => transmission end
    bcm2835_gpio_set(RPI_GPIO_P1_24);
}
