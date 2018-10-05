#include <bcm2835.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include "spi.h"
#include "udp_tmp.h"

int main(int argc, char **argv) {

    if (!bcm2835_init())
    {
        printf("bcm2835_init failed. Are you running as root??\n");
        exit(EXIT_FAILURE);
    }

    spi_init();


    int sock = make_socket();

    ip_addr target = make_addr("192.168.1.119", 6780);

    int counter = 0;

    while (1) {
        spi_mosi_data_t spi_mosi_data = {0};
        char sender_ip[100];

        int recv_len = 1;

        while(recv_len > 0) {
            recv_len = receive(sock, sender_ip, (void*)&spi_mosi_data, sizeof(spi_mosi_data_t));
        }


        spi_miso_data_t spi_miso_data = spi_transfer(spi_mosi_data);
        counter++;
        /*if(counter % 20 == 0) {
            printf("tick %u\n", spi_miso_data.tick);
            printf("odometer_steps %i\n", spi_miso_data.odometer_steps);
            printf("imu_yaw %u\n", spi_miso_data.imu_yaw);
            printf("\n");
        }*/

        send_to(sock, target, &spi_miso_data, sizeof(spi_miso_data_t));
        usleep(20000);
    }

    return 0;
}