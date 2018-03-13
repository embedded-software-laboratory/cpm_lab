#include "battery_monitor.h"
#include "remote_debug.h"
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"


void task_battery_monitor(void *pvParameters)
{
    while(1) {
        remote_debug_printf("battery/adc: %f\n", 0.010356 * sdk_system_adc_read()); // TODO calibration const as config var
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
