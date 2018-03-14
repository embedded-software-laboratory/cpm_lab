#include "battery_monitor.h"
#include "remote_debug.h"
#include "remote_config.h"
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"



float get_battery_voltage() {
    return CONFIG_VAR_battery_volts_per_adc_count * sdk_system_adc_read();
}

void task_battery_monitor(void *pvParameters)
{
    while(1) {
        float battery_volts = get_battery_voltage();

        if(battery_volts < 6.1) {
            remote_debug_printf("Low battery alert: %.3fV\n", battery_volts);
        }

        
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}
