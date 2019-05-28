/*
 * test_sequence.h
 *
 * Created: 04.12.2018 11:27:19
 *  Author: maczijewski
 */ 


#ifndef TEST_SEQUENCE_H_
#define TEST_SEQUENCE_H_


static inline void test_sequence(spi_mosi_data_t* spi_mosi_data, spi_miso_data_t* spi_miso_data)
{
	// servo current measurement loop
	//uint32_t tick;
	//uint8_t period	= 4;
	//uint8_t on		= 2;
	//
	//// 50 ticks per second
	//tick = spi_miso_data->tick;
	//
	//if(tick%(50*period)>50*on) {
		//spi_mosi_data->servo_command = 800;		// +800, -800
	//}
	//else {
		//spi_mosi_data->servo_command = -800;	// +800, -800
	//}
	//
	
	// systems test program
	spi_mosi_data->motor_mode = SPI_MOTOR_MODE_FORWARD;
	spi_mosi_data->motor_pwm = 60;
	
	// steer against yaw
	int32_t yaw = (int32_t)(spi_miso_data->imu_yaw);
	if(yaw > 4500) yaw -= 9000;
	
	spi_mosi_data->servo_command = 2*yaw;
	
	if(spi_mosi_data->servo_command >  400) spi_mosi_data->servo_command =  400;
	if(spi_mosi_data->servo_command < -400) spi_mosi_data->servo_command = -400;
	
	
	// motor + leds
	int32_t s = spi_miso_data->odometer_steps;
	
	
	
	spi_mosi_data->LED1_period_ticks = 1;
	spi_mosi_data->LED1_enabled_ticks = 0;
	spi_mosi_data->LED2_period_ticks = 1;
	spi_mosi_data->LED2_enabled_ticks = 0;
	spi_mosi_data->LED3_period_ticks = 1;
	spi_mosi_data->LED3_enabled_ticks = 0;
	spi_mosi_data->LED4_period_ticks = 1;
	spi_mosi_data->LED4_enabled_ticks = 0;
	
	if(s&1) spi_mosi_data->LED1_enabled_ticks = 1;
	if(s&2) spi_mosi_data->LED2_enabled_ticks = 1;
	if(s&4) spi_mosi_data->LED3_enabled_ticks = 1;
	if(s&8) spi_mosi_data->LED4_enabled_ticks = 1;
	
	
	
	if(s>0) spi_mosi_data->motor_mode = SPI_MOTOR_MODE_REVERSE;
	if(s<0) spi_mosi_data->motor_mode = SPI_MOTOR_MODE_FORWARD;
	
	if(s<0) s = -s;
	if(s > 200) s = 200;
	spi_mosi_data->motor_pwm = s;
}


#endif /* TEST_SEQUENCE_H_ */