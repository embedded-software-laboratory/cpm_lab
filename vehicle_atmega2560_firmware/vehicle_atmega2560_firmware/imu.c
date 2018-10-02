/*
 * imu.c
 *
 * Created: 27.09.2018 10:54:21
 *  Author: maczijewski
 */ 

#define BNO055_ADDRESS (0x28)


#define BNO055_CHIP_ID_ADDR  0x00
#define BNO055_CHIP_ID      (0xA0)
#define BNO055_EULER_H_LSB_ADDR (0x1A)
#define BNO055_EULER_H_MSB_ADDR (0x1B)
#define BNO055_OPR_MODE_ADDR     0X3D
#define BNO055_PWR_MODE_ADDR     0X3E
#define BNO055_POWER_MODE_NORMAL        0X00
#define BNO055_OPERATION_MODE_NDOF      0X0C
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR  0X28
#define BNO055_ACCEL_DATA_X_LSB_ADDR         0X08
#define BNO055_SYS_TRIGGER_ADDR              0X3F
#define BNO055_SYS_TRIGGER_RESET_SYSTEM  0b00100000


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include "twi.h"
#include "util.h"

bool imu_setup() {
	
	uint8_t buffer[10];
	uint8_t status = 0;
	
	_delay_ms(650); // Wait for the IMU to boot	
	
	// check chip ID
	buffer[0] = BNO055_CHIP_ID_ADDR;
	if(twi_writeTo(BNO055_ADDRESS, buffer, 1, true, false) != 0) {return false;}
	if(twi_readFrom(BNO055_ADDRESS, buffer, 1, true) != 1) {return false;}
	if(buffer[0] != BNO055_CHIP_ID) {return false;}
	_delay_ms(10);
	
	
	// Reset IMU
	buffer[0] = BNO055_SYS_TRIGGER_ADDR;
	buffer[1] = BNO055_SYS_TRIGGER_RESET_SYSTEM;
	status = twi_writeTo(BNO055_ADDRESS, buffer, 2, true, true);
	if(status != 0) {
		return false;
	}	
	_delay_ms(650); // Wait for the IMU to boot
		
		
	// set power mode
	buffer[0] = BNO055_PWR_MODE_ADDR;
	buffer[1] = BNO055_POWER_MODE_NORMAL;
	status = twi_writeTo(BNO055_ADDRESS, buffer, 2, true, true);
	if(status != 0) {
		return false;
	}	
	_delay_ms(10);
	
	
	// set operation mode
	buffer[0] = BNO055_OPR_MODE_ADDR;
	buffer[1] = BNO055_OPERATION_MODE_NDOF;
	status = twi_writeTo(BNO055_ADDRESS, buffer, 2, true, true);
	if(status != 0) {
		return false;
	}	
	
	return true;	
}

bool imu_read(uint16_t* imu_yaw, int16_t* imu_acceleration_forward, int16_t* imu_acceleration_left) {
	
	bool success_flag = true;
	uint8_t buffer[10];
	
	// read yaw
	buffer[0] = BNO055_EULER_H_LSB_ADDR;
	if(twi_writeTo(BNO055_ADDRESS, buffer, 1, true, false) != 0) success_flag = false;
	if(twi_readFrom(BNO055_ADDRESS, buffer, 2, true) != 2) success_flag = false;
	*imu_yaw = *((uint16_t*)(buffer));
	 
	_delay_us(50);
	
	// read acceleration
	buffer[0] = BNO055_ACCEL_DATA_X_LSB_ADDR;
	if(twi_writeTo(BNO055_ADDRESS, buffer, 1, true, false) != 0) success_flag = false;
	if(twi_readFrom(BNO055_ADDRESS, buffer, 4, true) != 4) success_flag = false;
	
	*imu_acceleration_left = -*((int16_t*)(buffer)); // TODO update axis mapping when the PCB arrives
	*imu_acceleration_forward = *((int16_t*)(buffer+2));
	
	return success_flag;
}