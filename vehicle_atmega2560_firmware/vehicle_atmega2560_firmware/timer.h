/*
 * timer.h
 *
 * Created: 08.01.2020 08:36:26
 *  Author: kloock
 */ 


#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "util.h"
#include "servo_timer.h"
#include "led.h"

void timer_setup();


#endif /* TIMER_H_ */