#include <bcm2835.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

void spi_init() {
    if (!bcm2835_spi_begin()) {
        printf("bcm2835_spi_begin failed. Are you running as root??\n");
        exit(EXIT_FAILURE);
    }

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

void spi_transfer() {
    while (1) {

        // CS low => transmission start
        bcm2835_gpio_clr(RPI_GPIO_P1_24);
        usleep(10);

        for (int i = 0; i < 27; ++i)
        {
            uint8_t send_data = 0;
            uint8_t read_data = bcm2835_spi_transfer(send_data);
            printf("%02x ", read_data);
            usleep(10);
        }
        printf(" --A\n");
        fflush(stdout);


        // CS high => transmission end
        bcm2835_gpio_set(RPI_GPIO_P1_24);
        usleep(10);
    }
}
