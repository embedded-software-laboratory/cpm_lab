#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ssid_config.h"
#include "lwip/api.h"
#include <string.h>
#include <stdarg.h>

#include "udp_debug.h"


void task_talker(void *pvParameters)
{
    int32_t i = 0;
    while(1) {
        float f = 0.123f * i;
        udp_debug_printf("counter %d, %f \n", i, f);
        vTaskDelay(pdMS_TO_TICKS(500));
        i++;
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

    init_udp_debug();

    xTaskCreate(task_udp_debug_sender, "task_udp_debug_sender", 512, NULL, 2, NULL);
    xTaskCreate(task_talker, "task_talker", 512, NULL, 2, NULL);
}
