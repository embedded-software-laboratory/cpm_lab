#include "remote_debug.h"
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ssid_config.h"
#include "lwip/api.h"
#include <string.h>


QueueHandle_t remote_debug_output_queue = NULL;

typedef struct {
    uint8_t length;
    char data[256];
} UdpPacket;

void init_remote_debug() {   
    if(remote_debug_output_queue == NULL) {
        remote_debug_output_queue = xQueueCreate(5, sizeof(UdpPacket));
        if(remote_debug_output_queue == NULL) {
            printf("%s: Error: xQueueCreate() failed\n", __FUNCTION__);
        }
    }
}


void task_remote_debug_sender(void *pvParameters) {
    while(1) {
        if(remote_debug_output_queue == NULL) {
            printf("%s: Error: remote_debug_output_queue is uninitialized\n", __FUNCTION__);
            vTaskDelete(NULL);
            return;
        }

        // wait for wifi connection
        while(sdk_wifi_station_get_connect_status() != STATION_GOT_IP) {
            printf("%s: Waiting for WiFi+IP...\n", __FUNCTION__);
            vTaskDelay(pdMS_TO_TICKS(2000));
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

        UdpPacket packet;
        struct netbuf* buf = NULL;
        void* data = NULL;
        bool loop = true;

        while(loop) {
            if(xQueueReceive(remote_debug_output_queue, &(packet), portMAX_DELAY) == pdPASS) {
                buf = netbuf_new();
                data = netbuf_alloc(buf, packet.length);
                memcpy (data, packet.data, packet.length);
                err = netconn_sendto(conn, buf, IP_ADDR_BROADCAST, 6780);

                if (err != ERR_OK) {
                    printf("%s : Could not send data! (%s)\n", __FUNCTION__, lwip_strerr(err));
                    netconn_delete(conn);
                    loop = false;
                }
                netbuf_delete(buf);
            }
        }
    }
}

void remote_debug_printf(const char* format, ...) {
    if(remote_debug_output_queue == NULL) return;
    UdpPacket packet;
    va_list argptr;
    va_start(argptr, format);
    vsnprintf(packet.data, 256, format, argptr);
    va_end(argptr);
    packet.length = strnlen(packet.data, 256);
    xQueueSendToBack(remote_debug_output_queue, (void*) &(packet), 0);
}