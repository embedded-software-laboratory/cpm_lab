/*
 * spi_packets.h
 *
 * Created: 24.09.2018 18:27:01
 *  Author: maczijewski
 */ 


#ifndef SPI_PACKETS_H_
#define SPI_PACKETS_H_



#define SPI_MOTOR_MODE_BRAKE 0
#define SPI_MOTOR_MODE_FORWARD 1
#define SPI_MOTOR_MODE_REVERSE 2
#define SPI_MOTOR_MODE_SPEEDCONTROL 3

#define SPI_BUFFER_SIZE 27

typedef struct
{
	int16_t target_speed;
	int16_t motor_pwm;
	int16_t servo_command;
	int16_t debugA;
	int16_t debugB;
	uint8_t motor_mode;
	uint8_t LED_bits;
	uint8_t CRC;
} spi_mosi_data_t;

_Static_assert(sizeof(spi_mosi_data_t) == 13, "spi_mosi_data_t unexpected size, not packed?");

typedef struct
{
	uint32_t tick;
	uint32_t odometer_steps;
	uint16_t imu_yaw;
	uint16_t imu_acceleration_forward;
	uint16_t imu_acceleration_left;
	int16_t speed;
	uint16_t battery_voltage;
	uint16_t motor_current;
	int16_t debugC;
	int16_t debugD;
	uint8_t status_flags;
	uint8_t CRC;
} spi_miso_data_t;


_Static_assert(sizeof(spi_miso_data_t) == 26, "spi_miso_data_t unexpected size, not packed?");


_Static_assert(sizeof(spi_mosi_data_t) + 1 <= SPI_BUFFER_SIZE, "SPI buffer too small");
_Static_assert(sizeof(spi_miso_data_t) + 1 <= SPI_BUFFER_SIZE, "SPI buffer too small");

#endif /* SPI_PACKETS_H_ */