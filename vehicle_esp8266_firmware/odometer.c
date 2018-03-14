#include "odometer.h"
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "remote_config.h"

#define ODOMETER_PIN 4

#define N_INTERRUPT_TIMES 4

uint32_t odometer_count = 0;
uint32_t interrupt_times[N_INTERRUPT_TIMES];
size_t interrupt_times_index = 0;

static void IRAM odometer_interrupt_handler(uint8_t gpio) {

    // remember when the odometer interrupt happens for speed calculation
    interrupt_times_index = (interrupt_times_index + 1) % N_INTERRUPT_TIMES;
    interrupt_times[interrupt_times_index] = timer_get_count(FRC2);
    timer_set_load(FRC2, 0);

    odometer_count++;
}

void init_odometer() {

    for (int i = 0; i < N_INTERRUPT_TIMES; ++i) { interrupt_times[i] = 0; }

    timer_set_interrupts(FRC2, false);
    timer_set_reload(FRC2, false);
    timer_set_run(FRC2, false);
    timer_set_divider(FRC2, TIMER_CLKDIV_16); // The timer runs at 5 MHz = 80MHz/16
    timer_set_run(FRC2, true);

    gpio_enable(ODOMETER_PIN, GPIO_INPUT);
    gpio_set_interrupt(ODOMETER_PIN, GPIO_INTTYPE_EDGE_NEG, odometer_interrupt_handler);
}

uint32_t get_average_ticks_per_odometer_count () {
    uint32_t sum = 0;
    for (int i = 0; i < N_INTERRUPT_TIMES; ++i) { sum += interrupt_times[i]; }
    return sum / N_INTERRUPT_TIMES;
}

float get_odometer_speed() {
    uint32_t ticks_per_count = get_average_ticks_per_odometer_count();
    uint32_t current_ticks = timer_get_count(FRC2);

    // if the current timer runs for a long time, the vehicle must have stopped
    if(current_ticks > 2*ticks_per_count) return 0;

    const float ticks_per_second = 5000000; // The timer runs at 5 MHz

    // speed in meters per second
    return CONFIG_VAR_meters_per_odometer_count * ticks_per_second / ((float)ticks_per_count);
}

uint32_t get_odometer_count() {
    return odometer_count;
}


float get_odometer_distance() {
    return CONFIG_VAR_meters_per_odometer_count * odometer_count;
}