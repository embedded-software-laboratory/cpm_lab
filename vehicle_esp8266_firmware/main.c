#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "esp/timer.h"
#include "ssid_config.h"
#include "lwip/api.h"
#include <string.h>
#include <stdarg.h>

#include "remote_debug.h"
#include "remote_config.h"
#include "servo_pwm.h"
#include "battery_monitor.h"
#include "odometer.h"
#include "speed_control.h"




void task_main(void *pvParameters) {

    /*while(1) {
        for (uint32_t i = 1000; i < 2000; i+=10) {
            servo_pwm_set_steering(i);
            servo_pwm_set_motor(CONFIG_VAR_motor_signal);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        for (int i = 2000; i > 1000; i-=10) {
            servo_pwm_set_steering(i);
            servo_pwm_set_motor(CONFIG_VAR_motor_signal);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }*/

    /*while(1) {
        //remote_debug_printf("odom %u counts , %f m\n", get_odometer_count(), get_odometer_distance());
        //remote_debug_printf("timer_get_count FRC2 %f \n", (timer_get_count(FRC2)/(5000000.0)));
        remote_debug_printf("get_odometer_speed %f \n", get_odometer_speed());
        //remote_debug_printf("frc2_count %u \n", frc2_count);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }*/

    float error_integral = 0;


    vTaskDelay(pdMS_TO_TICKS(1000));
    TickType_t previousWakeTime = xTaskGetTickCount();
    while(1) {

        float motor_signal = speed_control_get_motor_signal(CONFIG_VAR_reference_speed);

        servo_pwm_set_motor((uint32_t)motor_signal);



        /*remote_debug_printf("odom_speed %f reference_speed %f motor_signal %f error_integral %f \n", 
            odometer_speed, CONFIG_VAR_reference_speed, motor_signal, error_integral);*/

        vTaskDelayUntil(&previousWakeTime, pdMS_TO_TICKS(20));

    }

}




void user_init(void)
{
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());

    /** WiFi config **/
    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);

    /** Init modules **/
    init_remote_debug();
    init_servo_pwm();
    init_odometer();

    /** Disable 4th LED **/
    gpio_enable(0, GPIO_OUTPUT);
    gpio_write(0, 0);


    /** Start tasks **/
    xTaskCreate(task_remote_debug_sender, "task_remote_debug_sender", 512, NULL, 2, NULL);
    xTaskCreate(task_remote_config, "task_remote_config", 512, NULL, 2, NULL);
    xTaskCreate(task_main, "task_main", 512, NULL, 2, NULL);
    xTaskCreate(task_battery_monitor, "task_battery_monitor", 512, NULL, 2, NULL);
}
