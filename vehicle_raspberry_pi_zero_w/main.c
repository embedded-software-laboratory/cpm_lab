#include <bcm2835.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include "spi.h"

int main(int argc, char **argv) {

    if (!bcm2835_init())
    {
        printf("bcm2835_init failed. Are you running as root??\n");
        exit(EXIT_FAILURE);
    }

    spi_init();


    while (1) {
        spi_mosi_data_t spi_mosi_data = {0};
        spi_miso_data_t spi_miso_data = spi_transfer(spi_mosi_data);

        printf("tick %u\n", spi_miso_data.tick);
        printf("odometer_steps %i\n", spi_miso_data.odometer_steps);
        printf("imu_yaw %u\n", spi_miso_data.imu_yaw);
        printf("\n");
        usleep(60000);
    }

    return 0;
}