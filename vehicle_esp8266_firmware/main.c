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


#define MICROSECONDS_TO_TICKS 5

#define N_PWM_PINS 3
#define N_SCHEDULE_STEPS (N_PWM_PINS+1)

uint8_t schedule_step = 0;
uint8_t pin_numbers[] = {13, 12, 15};

uint32_t schedule_timer_ticks[N_SCHEDULE_STEPS] = {
    2000 * MICROSECONDS_TO_TICKS, 
    2000 * MICROSECONDS_TO_TICKS, 
    2000 * MICROSECONDS_TO_TICKS, 
   14000 * MICROSECONDS_TO_TICKS, 
};

uint8_t pin_values[N_PWM_PINS][N_SCHEDULE_STEPS];

static void IRAM frc1_interrupt_handler(void *arg) {
    timer_set_load(FRC1, schedule_timer_ticks[schedule_step]);
    for (int i = 0; i < N_PWM_PINS; ++i)
        gpio_write(pin_numbers[i], pin_values[i][schedule_step]);
    schedule_step = (schedule_step + 1) % N_SCHEDULE_STEPS;
}




/* For each pin, set the high-time. The maximum high-time is 20000 usec. */
void my_pwm_set_high_times(uint32_t* high_times_microseconds) {

    // bubblesort the times
    size_t permutation[N_PWM_PINS];
    for (int i = 0; i < N_PWM_PINS; ++i) 
        permutation[i] = i;

    while(1) {
        bool swapped = false;
        for (int i = 0; i < (N_PWM_PINS-1); ++i) {
            if(high_times_microseconds[permutation[i]] > high_times_microseconds[permutation[i+1]]) {
                size_t tmp = permutation[i];
                permutation[i] = permutation[i+1];
                permutation[i+1] = tmp;
                swapped = true;
            }
        }
        if(!swapped) break;
    }

    /** set time steps **/

    // first time step is just the first high time
    schedule_timer_ticks[0] = high_times_microseconds[permutation[0]] * MICROSECONDS_TO_TICKS;

    // subsequent time steps are the differences between successive high times
    for (int i = 0; i < (N_PWM_PINS-1); ++i) {
        uint32_t dt = high_times_microseconds[permutation[i+1]] - high_times_microseconds[permutation[i]];
        if(dt == 0) { schedule_timer_ticks[i+1] = 1; }
        else { schedule_timer_ticks[i+1] = dt * MICROSECONDS_TO_TICKS; }
    }

    // calculate the last time step, where all pins are low
    const uint32_t pwm_period_ticks = 20000 * MICROSECONDS_TO_TICKS;
    uint32_t last_falling_edge_tick = high_times_microseconds[permutation[N_PWM_PINS-1]] * MICROSECONDS_TO_TICKS;
    if(pwm_period_ticks > last_falling_edge_tick) {
        // set the remaining time to complete the PWM cycle
        schedule_timer_ticks[N_PWM_PINS] = pwm_period_ticks - last_falling_edge_tick;
    } else {
        schedule_timer_ticks[N_PWM_PINS] = 1;
    }

    /** set pin values **/

    // first time step: turn all pins on
    for (int i_pin = 0; i_pin < N_PWM_PINS; ++i_pin)
    {
        pin_values[i_pin][0] = 1;
    }

    // next steps: copy previous state, the turn one pin off
    for (int i_step = 1; i_step < N_SCHEDULE_STEPS; ++i_step)
    {
        for (int i_pin = 0; i_pin < N_PWM_PINS; ++i_pin)
        {
            pin_values[i_pin][i_step] = pin_values[i_pin][i_step-1];
        }

        int off_pin = permutation[i_step-1];
        pin_values[off_pin][i_step] = 0;
    }


}


void my_pwm_init() {
    uint32_t high_times[N_PWM_PINS] = { 2000, 7000, 3000, };

    my_pwm_set_high_times(high_times);

    for (int i = 0; i < N_PWM_PINS; ++i)
        gpio_enable(pin_numbers[i], GPIO_OUTPUT);

    _xt_isr_attach(INUM_TIMER_FRC1, frc1_interrupt_handler, NULL);
    timer_set_divider(FRC1, TIMER_CLKDIV_16);
    timer_set_load(FRC1, 1);
    timer_set_reload(FRC1, false);
    timer_set_interrupts(FRC1, true);
    timer_set_run(FRC1, true);
}



void task_pwm_test(void *pvParameters) {

}


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

    my_pwm_init();

    //xTaskCreate(task_remote_debug_sender, "task_remote_debug_sender", 512, NULL, 2, NULL);
    //xTaskCreate(task_remote_config, "task_remote_config", 512, NULL, 2, NULL);
    //xTaskCreate(task_talker, "task_talker", 512, NULL, 2, NULL);
    xTaskCreate(task_pwm_test, "task_pwm_test", 2048, NULL, 2, NULL);
}
