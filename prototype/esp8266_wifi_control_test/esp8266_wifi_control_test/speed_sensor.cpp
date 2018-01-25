#include "speed_sensor.h"


#define PIN_WHEELSPEED 4
#define WHEELSPEED_INTERRUPT_TIMES_SZ 100

unsigned long next_wheelspeed_interrupt_micors = 0;
unsigned long wheelspeed_interrupt_times[WHEELSPEED_INTERRUPT_TIMES_SZ];
int wheelspeed_interrupt_times_index = 0;


void pin_wheelspeed_interrupt_handler() {
  unsigned long now = micros();
  if (now > next_wheelspeed_interrupt_micors) {
    next_wheelspeed_interrupt_micors = now + 500;
    wheelspeed_interrupt_times[wheelspeed_interrupt_times_index] = now;
    wheelspeed_interrupt_times_index++;
    if (wheelspeed_interrupt_times_index >= WHEELSPEED_INTERRUPT_TIMES_SZ) {
      wheelspeed_interrupt_times_index = 0;
    }
  }
}


void SpeedSensor::setup() {
    pinMode(PIN_WHEELSPEED, INPUT);

    for (int i = 0; i < WHEELSPEED_INTERRUPT_TIMES_SZ; i++) {
      wheelspeed_interrupt_times[i] = 0;
    }

    attachInterrupt(digitalPinToInterrupt(PIN_WHEELSPEED), pin_wheelspeed_interrupt_handler, FALLING);
}

int SpeedSensor::get_speed() {
  int edge_count_in_time_range = 0;
  int i = wheelspeed_interrupt_times_index;
  unsigned long time_range_start = micros() - 20000;

  while (edge_count_in_time_range <= WHEELSPEED_INTERRUPT_TIMES_SZ) {
    i--;
    if (i < 0) i = WHEELSPEED_INTERRUPT_TIMES_SZ - 1;
    if (time_range_start > wheelspeed_interrupt_times[i]) break;
    edge_count_in_time_range++;
  }

  return edge_count_in_time_range;
}
