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
        spi_transfer();
        usleep(30000);
    }

    return 0;
}