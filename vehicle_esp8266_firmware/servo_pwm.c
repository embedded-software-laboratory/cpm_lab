#include "servo_pwm.h"


#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "ssid_config.h"
#include "lwip/api.h"
#include <string.h>
#include <stdarg.h>
#include <assert.h>


#define MICROSECONDS_TO_TICKS 5

#define PIN_MOTOR 15
#define PIN_STEERING_SERVO 16
#define N_PWM_PINS 2
#define N_SCHEDULE_STEPS (N_PWM_PINS+1)
uint8_t pin_numbers[N_PWM_PINS] = {PIN_MOTOR, PIN_STEERING_SERVO};
uint32_t schedule_timer_ticks[N_SCHEDULE_STEPS];
uint8_t pin_values[N_PWM_PINS][N_SCHEDULE_STEPS];
uint8_t schedule_step = 0;

static void IRAM frc1_interrupt_handler(void *arg) {
    timer_set_load(FRC1, schedule_timer_ticks[schedule_step]);
    for (int i = 0; i < N_PWM_PINS; ++i)
        gpio_write(pin_numbers[i], pin_values[i][schedule_step]);
    schedule_step = (schedule_step + 1) % N_SCHEDULE_STEPS;
}


/* For each pin, set the high-time. The maximum high-time is 20000 usec. */
static void servo_pwm_set_high_times(uint32_t* high_times_microseconds) {

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


void init_servo_pwm() {
    uint32_t high_times[N_PWM_PINS] = { 1500, 1500 };

    servo_pwm_set_high_times(high_times);

    for (int i = 0; i < N_PWM_PINS; ++i)
        gpio_enable(pin_numbers[i], GPIO_OUTPUT);

    _xt_isr_attach(INUM_TIMER_FRC1, frc1_interrupt_handler, NULL);
    timer_set_divider(FRC1, TIMER_CLKDIV_16);
    timer_set_load(FRC1, 1);
    timer_set_reload(FRC1, false);
    timer_set_interrupts(FRC1, true);
    timer_set_run(FRC1, true);
}



uint32_t high_times_global[N_PWM_PINS] = { 1500, 1500 };

void servo_pwm_set_steering(uint32_t signal_microseconds) {
    assert(pin_numbers[1] == PIN_STEERING_SERVO);
    if(signal_microseconds < 1000) signal_microseconds = 1000;
    else if(signal_microseconds > 2000) signal_microseconds = 2000;
    high_times_global[1] = signal_microseconds;
    servo_pwm_set_high_times(high_times_global);
}

void servo_pwm_set_motor(uint32_t signal_microseconds) {
    assert(pin_numbers[0] == PIN_MOTOR);
    if(signal_microseconds < 1000) signal_microseconds = 1000;
    else if(signal_microseconds > 2000) signal_microseconds = 2000;
    high_times_global[0] = signal_microseconds;
    servo_pwm_set_high_times(high_times_global);
}

