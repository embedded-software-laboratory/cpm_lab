#include "i2c_bus.h"



void task_i2c_bus(void *pvParameters) {

    vTaskDelay(pdMS_TO_TICKS(500));
    //TickType_t previousWakeTime = xTaskGetTickCount();
    //vTaskDelayUntil(&previousWakeTime, pdMS_TO_TICKS(10));


    while(1) {
        


        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}