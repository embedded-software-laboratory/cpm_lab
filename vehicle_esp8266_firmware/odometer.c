#include "odometer.h"
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "remote_config.h"

#define ODOMETER_PIN 4

uint32_t odometer_count = 0;

static void IRAM odometer_interrupt_handler(uint8_t gpio)
{
    odometer_count++;
}


void init_odometer() {
    gpio_enable(ODOMETER_PIN, GPIO_INPUT);
    gpio_set_interrupt(ODOMETER_PIN, GPIO_INTTYPE_EDGE_NEG, odometer_interrupt_handler);
}

uint32_t get_odometer_count() {
    return odometer_count;
}


float get_odometer_distance() {
    return CONFIG_VAR_meters_per_odometer_count * odometer_count;
}