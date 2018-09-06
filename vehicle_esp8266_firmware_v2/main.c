#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "esp/spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "esp/timer.h"
#include "ssid_config.h"


void task_main(void *pvParameters) {


    vTaskDelay(pdMS_TO_TICKS(1000));
    //TickType_t previousWakeTime = xTaskGetTickCount();
    //vTaskDelayUntil(&previousWakeTime, pdMS_TO_TICKS(10));

    uint16_t servo = 0;

    uint8_t i = 0;
    while(1) {

        if(i==0) {
            servo++;
        }

        uint8_t message[] = {'a', 'S', 20, (uint8_t)((servo>>4)&0xff), 11, 'Y', 'j'};
        uint8_t recvd = spi_transfer_8(1, message[i]);

        i = (i+1)%7;

        //printf("Hello %6i, %6i\n", message[i], recvd);


        sdk_os_delay_us(50);
    }
}


void user_init(void)
{
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());

    /** WiFi config **/
    //struct sdk_station_config config = {
    //    .ssid = WIFI_SSID,
    //    .password = WIFI_PASS,
    //};
    //sdk_wifi_set_opmode(STATION_MODE);
    //sdk_wifi_station_set_config(&config);

    /** Init modules **/
    if(!spi_init(1, 0, SPI_FREQ_DIV_500K, true, SPI_BIG_ENDIAN, true)) {
        printf("Error in spi_init()\n");
    }

    /** Start tasks **/
    xTaskCreate(task_main, "task_main", 512, NULL, 2, NULL);
}
