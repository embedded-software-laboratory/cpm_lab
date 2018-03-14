#include "remote_command.h"
#include "remote_config.h"
#include "remote_debug.h"

#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lwip/api.h"
#include <string.h>


float command_speed = 0;
float command_curvature = 0;
uint8_t command_TTL = 0;

bool get_speed_and_curvature_command(float* command_speed_out, float* command_curvature_out) {

    // TTL countdown: If TTL goes to zero, we are not receiving new commands.
    if(command_TTL > 0) {
        command_TTL--;
        *command_speed_out = command_speed;
        *command_curvature_out = command_curvature;
        return true;
    }

    return false;
}

void decode_command_message(char* message, uint16_t length) {
    if(length > 1) {
        switch(message[0]) {
            case 'c': // Direct control message
            {
                if(length == 9) {
                    memcpy(&command_speed, message+1, sizeof(float));
                    memcpy(&command_curvature, message+5, sizeof(float));
                    command_TTL = 4;
                }
            }
            break;
            default: break;
        }
    }
}

void task_remote_command(void *pvParameters) {
    while(1) {
        // wait for wifi connection
        while(sdk_wifi_station_get_connect_status() != STATION_GOT_IP) {
            printf("%s: Waiting for WiFi+IP...\n", __FUNCTION__);
            vTaskDelay(pdMS_TO_TICKS(500));
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

        while(1) {

            // Receive message
            struct netbuf *netbuf;
            err = netconn_recv(conn, &netbuf);
            if(err == ERR_OK && netbuf_len(netbuf) < 256) {
                char message[256];
                netbuf_copy(netbuf, message, netbuf_len(netbuf));
                message[netbuf_len(netbuf)] = 0;

                decode_command_message(message, netbuf_len(netbuf));
            }
            netbuf_delete(netbuf);
        }
    }
}