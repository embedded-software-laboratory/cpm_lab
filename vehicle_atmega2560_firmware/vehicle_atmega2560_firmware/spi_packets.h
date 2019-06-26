/*
 * spi_packets.h
 *
 * Created: 24.09.2018 18:27:01
 *  Author: maczijewski
 */ 


#ifndef SPI_PACKETS_H_
#define SPI_PACKETS_H_


#include <stdint.h>


#define SPI_MOTOR_MODE_BRAKE 0
#define SPI_MOTOR_MODE_FORWARD 1
#define SPI_MOTOR_MODE_REVERSE 2

#define SPI_BUFFER_SIZE 27


typedef struct
{ // 15 bytes
	uint32_t pi_tick;
	int16_t motor_pwm;
	int16_t servo_command;
	uint16_t CRC;
	uint8_t motor_mode;
	uint8_t LED1_enabled;
	uint8_t LED2_enabled;
	uint8_t LED3_enabled;
	uint8_t LED4_enabled;
} __attribute__((packed)) spi_mosi_data_t;


typedef struct
{ // 27 bytes
	uint32_t tick;
	int32_t odometer_steps;
	uint16_t imu_yaw;
	int16_t imu_yaw_rate;
	int16_t imu_acceleration_forward;
	int16_t imu_acceleration_left;
	int16_t imu_acceleration_up;
	int16_t speed;
	uint16_t battery_voltage;
	uint16_t motor_current;
	uint16_t CRC;
	uint8_t status_flags;
	/*
	 * status_flags:
	 * Bit 7: reserved
	 * Bit 6: reserved
	 * Bit 5: reserved
	 * Bit 4: reserved
	 * Bit 3: reserved
	 * Bit 2: reserved
	 * Bit 1: SPI status, 0 -> previous mosi packet correctly received, 1 -> fault
	 * Bit 0: IMU status, 0 -> OK, 1 -> fault
	*/
} __attribute__((packed)) spi_miso_data_t;


#ifdef __cplusplus
static_assert(sizeof(spi_mosi_data_t) == 15, "spi_mosi_data_t unexpected size, not packed?");
static_assert(sizeof(spi_miso_data_t) == 27, "spi_miso_data_t unexpected size, not packed?");
static_assert(sizeof(spi_mosi_data_t) <= SPI_BUFFER_SIZE, "SPI buffer too small");
static_assert(sizeof(spi_miso_data_t) <= SPI_BUFFER_SIZE, "SPI buffer too small");
#else
_Static_assert(sizeof(spi_mosi_data_t) == 15, "spi_mosi_data_t unexpected size, not packed?");
_Static_assert(sizeof(spi_miso_data_t) == 27, "spi_miso_data_t unexpected size, not packed?");
_Static_assert(sizeof(spi_mosi_data_t) <= SPI_BUFFER_SIZE, "SPI buffer too small");
_Static_assert(sizeof(spi_miso_data_t) <= SPI_BUFFER_SIZE, "SPI buffer too small");
#endif


#endif /* SPI_PACKETS_H_ */