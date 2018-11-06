#include <bcm2835.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include "spi.h"

volatile int dummy = 0;

void busy_wait(int n) {
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

    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE0);
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_4096);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);

    // enable CS pin
    bcm2835_gpio_fsel(RPI_GPIO_P1_24, BCM2835_GPIO_FSEL_OUTP);
    usleep(1000);

    // Default CS to high
    bcm2835_gpio_set(RPI_GPIO_P1_24);
    usleep(1000);
}

spi_miso_data_t spi_transfer(spi_mosi_data_t spi_mosi_data) {

    uint8_t SPI_recv_buffer[SPI_BUFFER_SIZE];
    uint8_t* mosi_data_ptr = (uint8_t*)(&spi_mosi_data);

    // CS low => transmission start
    bcm2835_gpio_clr(RPI_GPIO_P1_24);

    busy_wait(5000);

    for (int i = 0; i < SPI_BUFFER_SIZE; ++i)
    {
        SPI_recv_buffer[i] = bcm2835_spi_transfer(mosi_data_ptr[i]);
        busy_wait(5000);
    }

    // CS high => transmission end
    bcm2835_gpio_set(RPI_GPIO_P1_24);

    spi_miso_data_t spi_miso_data;

    memcpy(&spi_miso_data, SPI_recv_buffer+1, sizeof(spi_miso_data_t));
    return spi_miso_data;    
}
