#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "esp/spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "esp/timer.h"
#include "ssid_config.h"
#include "spi_attiny.h"
#include "i2c_bus.h"



void task_main(void *pvParameters) {


    vTaskDelay(pdMS_TO_TICKS(2000));
    //TickType_t previousWakeTime = xTaskGetTickCount();
    //vTaskDelayUntil(&previousWakeTime, pdMS_TO_TICKS(10));


    while(1) {

        printf("ADC %8i\n", attiny_get_adc_value());

        attiny_set_led(1);
        attiny_set_driving_commands(MOTOR_DIRECTION_BRAKE, 0, 100);
        vTaskDelay(pdMS_TO_TICKS(1000));

        attiny_set_led(0);
        attiny_set_driving_commands(MOTOR_DIRECTION_BRAKE, 0, 140);
        vTaskDelay(pdMS_TO_TICKS(1000));

        attiny_set_led(1);
        attiny_set_driving_commands(MOTOR_DIRECTION_FORWARD, 40, 125);
        vTaskDelay(pdMS_TO_TICKS(1000));

        attiny_set_led(0);
        attiny_set_driving_commands(MOTOR_DIRECTION_REVERSE, 40, 125);
        vTaskDelay(pdMS_TO_TICKS(1000));

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
    init_spi_attiny();

    /** Start tasks **/
    xTaskCreate(task_main, "task_main", 512, NULL, 2, NULL);
    xTaskCreate(task_spi_attiny, "task_spi_attiny", 512, NULL, 2, NULL);
    xTaskCreate(task_i2c_bus, "task_i2c_bus", 512, NULL, 2, NULL);
}
