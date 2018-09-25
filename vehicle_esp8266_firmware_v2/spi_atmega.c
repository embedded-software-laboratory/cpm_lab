#include "spi_atmega.h"
#include "esp/spi.h"
#include "espressif/esp_common.h"
#include <stdbool.h>


void spi_atmega_setup() {

    gpio_enable(15, GPIO_OUTPUT);
    gpio_write(15, 1);
    vTaskDelay(pdMS_TO_TICKS(400));

    if(!spi_init(1, 0, SPI_FREQ_DIV_2M, true, SPI_BIG_ENDIAN, true)) {
        printf("Error in spi_init()\n");
    }
}


spi_miso_data_t spi_atmega_exchange(spi_mosi_data_t send_data) {

    uint8_t miso_buffer[SPI_BUFFER_SIZE];
    uint8_t mosi_buffer[SPI_BUFFER_SIZE];

    uint8_t* send_data_ptr = (uint8_t*)(&send_data);

    for (int i = 0; i < sizeof(spi_mosi_data_t); ++i)
    {
        mosi_buffer[i] = send_data_ptr[i];
    }

    gpio_write(15, 0);
    sdk_os_delay_us(2);
    for (int i = 0; i < SPI_BUFFER_SIZE; ++i)
    {
        miso_buffer[i] = spi_transfer_8(1, mosi_buffer[i]);
        sdk_os_delay_us(2);
    }
    gpio_write(15, 1);



    spi_miso_data_t recv_data;
    uint8_t* recv_data_ptr = (uint8_t*)(&recv_data);
    for (int i = 0; i < sizeof(spi_miso_data_t); ++i)
    {
        recv_data_ptr[i] = miso_buffer[i+1];
        //                              ^
        // off by one, because the SPI slave response is delayed
    }
    return recv_data;
}