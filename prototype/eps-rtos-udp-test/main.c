#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ssid_config.h"
#include "lwip/api.h"
#include <string.h>


void task1(void *pvParameters)
{
    while(1) {
        // wait for wifi connection
        while(sdk_wifi_station_get_connect_status() != STATION_GOT_IP) {
            printf("Waiting for WiFi+IP...\n");
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        // UDP socket
        err_t err;
        struct netconn* conn = netconn_new(NETCONN_UDP);
        err = netconn_bind(conn, IP_ADDR_ANY, 6780);
        if (err != ERR_OK) {
            netconn_delete(conn);
            printf("%s : Could not bind! (%s)\n", __FUNCTION__, lwip_strerr(err));
            continue;
        }

        int i = 0;
        while(1) {

            char msg[100];
            sprintf(msg, "counter %d \n", i);
            printf("%s", msg);

            struct netbuf* buf = netbuf_new();
            void* data = netbuf_alloc(buf, strlen(msg));

            memcpy (data, msg, strlen(msg));
            err = netconn_sendto(conn, buf, IP_ADDR_BROADCAST, 6780);

            if (err != ERR_OK) {
                printf("%s : Could not send data!!! (%s)\n", __FUNCTION__, lwip_strerr(err));
                continue;
            }
            netbuf_delete(buf);

            vTaskDelay(pdMS_TO_TICKS(500));
            i++;
        }
    }
}

void user_init(void)
{
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());

    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };

    /* required to call wifi_set_opmode before station_set_config */
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);


    xTaskCreate(task1, "tsk1", 256, NULL, 2, NULL);
}
