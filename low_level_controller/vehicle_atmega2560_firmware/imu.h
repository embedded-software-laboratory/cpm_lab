/**
 * \file imu.h
 *
 * \author maczijewski
 * \date Created: 27.09.2018 10:54:07
 * 
 * \brief This module provides an interface for setting up and reading values of the inertial measurement unit (IMU).
 * 
 * For details on the used IMU and the implementation see \link imu.c \endlink.
 * 
 * \ingroup low_level_controller
 */ 


#ifndef IMU_H_
#define IMU_H_


#include <stdbool.h>
#include <stdint.h>

/**
 * \brief Starts IMU and configures its power mode, operation mode, and sensors.
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
bool imu_setup();

/**
 * \brief Reads the sensor values currently provided by the IMU.
 * \param imu_yaw value of the yaw relative to the initial orientation. This result is not directly
 * 				  read from the IMU but accumulated based on the yaw_rate.
 * \param imu_yaw_rate rate with which the yaw changes
 * \param imu_acceleration_forward acceleration in forward direction
 * \param imu_acceleration_left acceleration to the left
 * \param imu_acceleration_up acceleration upwards
 * 
 * \author maczijewski
 * \ingroup low_level_controller
 */
bool imu_read(
	uint16_t* imu_yaw,
	int16_t* imu_yaw_rate,
	int16_t* imu_acceleration_forward,
	int16_t* imu_acceleration_left,
	int16_t* imu_acceleration_up
);


#endif /* IMU_H_ */