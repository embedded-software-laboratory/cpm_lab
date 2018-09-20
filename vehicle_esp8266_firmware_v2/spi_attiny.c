#include "spi_attiny.h"
#include "esp/spi.h"
#include "espressif/esp_common.h"
#include <stdbool.h>

#define SPI_PACKAGE_SIZE 7


typedef union
{
    uint8_t buffer[SPI_PACKAGE_SIZE];
    struct
    {
        uint8_t marker_a;
        uint8_t motor_direction;
        uint8_t motor_command;
        uint8_t servo_command;
        uint8_t led_command;
        uint8_t marker_Y;
        uint8_t marker_j;
    } data;
} spi_mosi_package_t;

typedef union
{
    uint8_t buffer[SPI_PACKAGE_SIZE];
    struct
    {
        uint8_t marker_f;
        uint8_t marker_P;
        uint8_t adc_h;
        uint8_t adc_l;
        uint8_t marker_K;
        uint8_t marker_4;
        uint8_t marker_q;
    } data;
} spi_miso_package_t;



spi_mosi_package_t spi_mosi_package;
uint16_t adc_value = 0;


void attiny_set_driving_commands(
    uint8_t motor_direction, 
    uint8_t motor_command, 
    uint8_t servo_command)
{
    spi_mosi_package.data.motor_direction = motor_direction;
    spi_mosi_package.data.motor_command = motor_command;
    spi_mosi_package.data.servo_command = servo_command;
}

void attiny_set_led(uint8_t led_command) 
{
    spi_mosi_package.data.led_command = led_command;
}

uint16_t attiny_get_adc_value()
{
    return adc_value;
}

void task_spi_attiny(void *pvParameters) {


    vTaskDelay(pdMS_TO_TICKS(400));

    if(!spi_init(1, 0, SPI_FREQ_DIV_500K, true, SPI_BIG_ENDIAN, true)) {
        printf("Error in spi_init()\n");
    }


    TickType_t previousWakeTime = xTaskGetTickCount();

    // Default init spi message
    spi_mosi_package.data.marker_a = 'a';
    spi_mosi_package.data.motor_direction = 0;
    spi_mosi_package.data.motor_command = 0;
    spi_mosi_package.data.servo_command = 125;
    spi_mosi_package.data.led_command = 0;
    spi_mosi_package.data.marker_Y = 'Y';
    spi_mosi_package.data.marker_j = 'j';

    bool insert_shift_byte = false;

    while(1) {

        // insert extra byte if synchronization is lost. 
        // this should eventually restore synchronization
        if(insert_shift_byte) { 
            insert_shift_byte = false;
            spi_transfer_8(1, 0);
        }

        spi_miso_package_t spi_miso_package;

        for (int i = 0; i < SPI_PACKAGE_SIZE; ++i)
        {
            spi_miso_package.buffer[(i+SPI_PACKAGE_SIZE-1)%SPI_PACKAGE_SIZE] = spi_transfer_8(1, spi_mosi_package.buffer[i]);
            sdk_os_delay_us(50);
        }


        // received message aligned?
        if(spi_miso_package.data.marker_f == 'f'
        && spi_miso_package.data.marker_P == 'P'
        && spi_miso_package.data.marker_K == 'K'
        && spi_miso_package.data.marker_4 == '4'
        && spi_miso_package.data.marker_q == 'q')
        {
            adc_value = (((uint16_t)spi_miso_package.data.adc_h) << 8)
                       | ((uint16_t)spi_miso_package.data.adc_l);
        } else {
            insert_shift_byte = true;

            printf("Error in SPI communication with ATtiny, received:\n");
            for (int i = 0; i < SPI_PACKAGE_SIZE; ++i)
            {
                printf("%8i", spi_miso_package.buffer[i]);
            }
            printf("\n");
        }

        vTaskDelayUntil(&previousWakeTime, pdMS_TO_TICKS(10));
    }
}