/*
 * imu.h
 *
 * Created: 27.09.2018 10:54:07
 *  Author: maczijewski
 */ 


#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>
#include <stdint.h>

bool imu_setup();
bool imu_read(uint16_t* imu_yaw, uint16_t* imu_acceleration_forward, uint16_t* imu_acceleration_left);


#endif /* IMU_H_ */