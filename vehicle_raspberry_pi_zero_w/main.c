#include <bcm2835.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include "spi.h"
#include "udp_tmp.h"
#include "../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/crc.h"
#include "../vehicle_atmega2560_firmware/vehicle_atmega2560_firmware/crc.c"

int main(int argc, char **argv) {
    crcInit();

    if (!bcm2835_init())
    {
        printf("bcm2835_init failed. Are you running as root??\n");
        exit(EXIT_FAILURE);
    }

    spi_init();


    int sock = make_socket();

    ip_addr target = make_addr("192.168.1.119", 6780);



    spi_mosi_data_t spi_mosi_data = {0};

    int spi_mosi_data_countdown = 0;

    while (1) {
        spi_mosi_data_t spi_mosi_data_new = {0};
        char sender_ip[100];

        int recv_len = 0;


        while(1) {
            int recv_len_new = receive(sock, sender_ip, (void*)&spi_mosi_data_new, sizeof(spi_mosi_data_t));
            if(recv_len_new <= 0) {
                break;
            }
            recv_len = recv_len_new;
        }
        //printf("recv_len %i\n", recv_len);

        if(recv_len == sizeof(spi_mosi_data_t)) {
            // new packet arrived
            // CRC check
            uint16_t crc_revcd = spi_mosi_data_new.CRC;
            spi_mosi_data_new.CRC = 0;
            uint16_t crc_calculated = crcFast((uint8_t*)&spi_mosi_data_new, sizeof(spi_mosi_data_t));
            spi_mosi_data_new.CRC = crc_revcd;

            //printf("crc %u, %u\n", crc_revcd, crc_calculated);
            if(crc_revcd == crc_calculated) {
                spi_mosi_data = spi_mosi_data_new;
                spi_mosi_data_countdown = 10;
                //printf("CRC correct\n");
            } 
            else {
                printf("CRC error\n");
            }


        }

        if(spi_mosi_data_countdown > 0) {
            spi_mosi_data_countdown--;
        }
        else {            
            memset(&spi_mosi_data, 0, sizeof(spi_mosi_data_t));
        }

        spi_miso_data_t spi_miso_data = spi_transfer(spi_mosi_data);
        
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