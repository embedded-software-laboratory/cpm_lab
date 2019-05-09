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

bool imu_read(
	uint16_t* imu_yaw,
	int16_t* imu_yaw_rate,
	int16_t* imu_acceleration_forward,
	int16_t* imu_acceleration_left,
	int16_t* imu_acceleration_up
);



#endif /* IMU_H_ */