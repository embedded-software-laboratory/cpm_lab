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

#define SPI_BUFFER_SIZE 28

#define LED1_BLINK_FAST (0<<0)
#define LED1_BLINK_SLOW (1<<0)
#define LED1_OFF        (2<<0)
#define LED1_ON         (3<<0)

#define LED2_BLINK_FAST (0<<2)
#define LED2_BLINK_SLOW (1<<2)
#define LED2_OFF        (2<<2)
#define LED2_ON         (3<<2)

#define LED3_BLINK_FAST (0<<4)
#define LED3_BLINK_SLOW (1<<4)
#define LED3_OFF        (2<<4)
#define LED3_ON         (3<<4)

#define LED4_BLINK_FAST (0<<6)
#define LED4_BLINK_SLOW (1<<6)
#define LED4_OFF        (2<<6)
#define LED4_ON         (3<<6)

typedef struct
{
	int16_t motor_pwm;
	int16_t servo_command;
	int16_t debugA;
	int16_t debugB;
	uint16_t CRC;
	uint8_t motor_mode;
	uint8_t LED_bits;
} __attribute__((packed)) spi_mosi_data_t;


typedef struct
{
	uint32_t tick;
	int32_t odometer_steps;
	uint16_t imu_yaw;
	int16_t imu_acceleration_forward;
	int16_t imu_acceleration_left;
	int16_t speed;
	uint16_t battery_voltage;
	uint16_t motor_current;
	int16_t debugC;
	int16_t debugD;
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
	 * Bit 1: reserved
	 * Bit 0: IMU status, 0 -> OK, 1 -> fault
	*/
} __attribute__((packed)) spi_miso_data_t;



#ifdef __cplusplus
static_assert(sizeof(spi_mosi_data_t) == 12, "spi_mosi_data_t unexpected size, not packed?");
static_assert(sizeof(spi_miso_data_t) == 27, "spi_miso_data_t unexpected size, not packed?");
static_assert(sizeof(spi_mosi_data_t) + 1 <= SPI_BUFFER_SIZE, "SPI buffer too small");
static_assert(sizeof(spi_miso_data_t) + 1 <= SPI_BUFFER_SIZE, "SPI buffer too small");
#else
_Static_assert(sizeof(spi_mosi_data_t) == 12, "spi_mosi_data_t unexpected size, not packed?");
_Static_assert(sizeof(spi_miso_data_t) == 27, "spi_miso_data_t unexpected size, not packed?");
_Static_assert(sizeof(spi_mosi_data_t) + 1 <= SPI_BUFFER_SIZE, "SPI buffer too small");
_Static_assert(sizeof(spi_miso_data_t) + 1 <= SPI_BUFFER_SIZE, "SPI buffer too small");
#endif

#endif /* SPI_PACKETS_H_ */