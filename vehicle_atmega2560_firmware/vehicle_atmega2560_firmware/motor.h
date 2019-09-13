/*
 * motor.h
 *
 * Created: 20.09.2018 10:02:02
 *  Author: maczijewski
 */ 


#ifndef MOTOR_H_
#define MOTOR_H_


#include <stdint.h>


#define MOTOR_DIRECTION_BRAKE 0
#define MOTOR_DIRECTION_FORWARD 1
#define MOTOR_DIRECTION_REVERSE 2

void motor_set_direction(uint8_t direction);
void motor_set_duty(uint16_t duty); // values from 0 to 400
void motor_setup();


#endif /* MOTOR_H_ */