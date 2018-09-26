#include "wifi_command.h"

#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "lwip/api.h"
#include <string.h>
#include "task.h"
#include "master_ip.h"

static spi_mosi_data_t spi_mosi_data_glob;
static int command_TTL = 0;

bool wifi_get_command(spi_mosi_data_t* out) {
    *out = spi_mosi_data_glob;
    command_TTL--;
    return command_TTL > 0;
}


void task_wifi_command(void *pvParameters) {
    while(1) {
        // wait for wifi connection
        while(sdk_wifi_station_get_connect_status() != STATION_GOT_IP) {
            printf("%s: Waiting for WiFi+IP...\n", __FUNCTION__);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }


        // UDP socket
        err_t err;
        struct netconn* conn = netconn_new(NETCONN_UDP);
        err = netconn_bind(conn, IP_ADDR_ANY, 6783);
        if (err != ERR_OK) {
            netconn_delete(conn);
            printf("%s : Could not bind! (%s)\n", __FUNCTION__, lwip_strerr(err));
            continue;
        }

        ip_addr_t master_address;
        ipaddr_aton(MASTER_IP, &master_address);

        while(1) {

            // Receive message
            struct netbuf *netbuf;
            err = netconn_recv(conn, &netbuf);
            printf("netbuf_len(netbuf) %i\n", netbuf_len(netbuf));
            if(err == ERR_OK 
                && netbuf_len(netbuf) == sizeof(spi_mosi_data_t) 
                && ip_addr_cmp(&master_address, &(netbuf->addr))) 
            {
                spi_mosi_data_t spi_mosi_data;
                netbuf_copy(netbuf, (void*)(&spi_mosi_data), sizeof(spi_mosi_data_t) );
                spi_mosi_data_glob = spi_mosi_data;
                command_TTL = 10;
            }
            netbuf_delete(netbuf);
        }
    }
}