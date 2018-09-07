#include "FreeRTOS.h"
#include "task.h"

#define MOTOR_DIRECTION_BRAKE 0
#define MOTOR_DIRECTION_FORWARD 1
#define MOTOR_DIRECTION_REVERSE 2

void attiny_set_driving_commands(uint8_t motor_direction, uint8_t motor_command, uint8_t servo_command);
void attiny_set_led(uint8_t led_command);
uint16_t attiny_get_adc_value();
void task_spi_attiny(void *pvParameters);