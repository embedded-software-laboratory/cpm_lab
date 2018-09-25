#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "esp/spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "esp/timer.h"
#include "ssid_config.h"
#include "spi_atmega.h"
#include "wifi_telemetry.h"
#include "wifi_command.h"


void task_main(void *pvParameters) {


    vTaskDelay(pdMS_TO_TICKS(5000));
    //TickType_t previousWakeTime = xTaskGetTickCount();
    //vTaskDelayUntil(&previousWakeTime, pdMS_TO_TICKS(10));

    uint8_t count = 0;

    while(1) {



        spi_mosi_data_t send_data;

        send_data.LED_bits = count;
        send_data.debugA = count;
        //if((count>>4)&1)
            send_data.servo_command = 3200;
        //else
        //    send_data.servo_command = 2800;

        spi_miso_data_t recv_data = spi_atmega_exchange(send_data);

        //printf("tick %i debugA %i debugC %i speed %i\n", recv_data.tick, send_data.debugA, recv_data.debugC, recv_data.speed);

        //char msg[] = "Hallo\n";

        wifi_telemetry_send((void*)(&recv_data), sizeof(spi_miso_data_t));

        count++;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


void user_init(void)
{
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());

    /** WiFi config **/
    sdk_wifi_set_opmode(0); // disable
    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);

    /** Init modules **/
    spi_atmega_setup();
    wifi_telemetry_setup();

    /** Start tasks **/
    xTaskCreate(task_main, "task_main", 512, NULL, 2, NULL);
    xTaskCreate(task_wifi_command, "task_wifi_command", 512, NULL, 2, NULL);
}

