/*
 * test_sequence.h
 *
 * Created: 04.12.2018 11:27:19
 *  Author: maczijewski
 * Modified: 05.31.2019 11:32:19
 *  Author: cfrauzem
 */ 


#ifndef TEST_SEQUENCE_H_
#define TEST_SEQUENCE_H_


typedef enum
{
	TEST_LED,
	TEST_SERVO_CURRENT,
	TEST_SERVO_ENABLE,
	TEST_SYSTEM,
	TEST_NONE
} TestSequenceName;

void test_sequence(spi_mosi_data_t* spi_mosi_data, spi_miso_data_t* spi_miso_data, TestSequenceName testSequenceName);


#endif /* TEST_SEQUENCE_H_ */