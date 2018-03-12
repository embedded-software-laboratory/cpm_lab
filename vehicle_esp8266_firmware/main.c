#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ssid_config.h"
#include "lwip/api.h"
#include <string.h>
#include <stdarg.h>

#include "remote_debug.h"
#include "remote_config.h"


void task_talker(void *pvParameters)
{
    while(1) {
        remote_debug_printf("         talker:  floatEEEE %f\n", CONFIG_VAR_floatE);
        vTaskDelay(pdMS_TO_TICKS(2000));
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

    init_remote_debug();

    xTaskCreate(task_remote_debug_sender, "task_remote_debug_sender", 512, NULL, 2, NULL);
    xTaskCreate(task_remote_config, "task_remote_config", 512, NULL, 2, NULL);
    xTaskCreate(task_talker, "task_talker", 512, NULL, 2, NULL);
}
