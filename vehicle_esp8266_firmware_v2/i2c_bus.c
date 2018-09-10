#include "i2c_bus.h"
#include <i2c/i2c.h>
#include "espressif/esp_common.h"
#include "esp/uart.h"


#define SDA_PIN (2)
#define SCL_PIN (0)
#define BNO055_ADDRESS (0x28)
#define BNO055_CHIP_ID_ADDR (0x00)
#define BNO055_CHIP_ID      (0xA0)
#define BNO055_EULER_H_LSB_ADDR (0x1A)
#define BNO055_EULER_H_MSB_ADDR (0x1B)


void writeByte(uint8_t slave_addr, uint8_t register_addr, uint8_t data_byte) {
    if(i2c_slave_write(0, slave_addr, &register_addr, &data_byte, 1) != 0) {
        printf("Error in i2c_slave_write()\n");
    }
}

uint8_t readByte(uint8_t slave_addr, uint8_t register_addr) {
    uint8_t data_byte = 0;
    if(i2c_slave_read(0, slave_addr, &register_addr, &data_byte, 1)) {
        printf("Error in i2c_slave_read()\n");
    }
    return data_byte;
}


void task_i2c_bus(void *pvParameters) {

    vTaskDelay(pdMS_TO_TICKS(500));
    //TickType_t previousWakeTime = xTaskGetTickCount();
    //vTaskDelayUntil(&previousWakeTime, pdMS_TO_TICKS(10));

    if(i2c_init(0, SCL_PIN, SDA_PIN, I2C_FREQ_100K) != 0) {
        printf("Error in i2c_init()\n");
    }

    i2c_set_clock_stretch(0, 80000);

    vTaskDelay(1);

    uint8_t bno055_chip_id_val = readByte(BNO055_ADDRESS, BNO055_CHIP_ID_ADDR);
    if(bno055_chip_id_val != BNO055_CHIP_ID){
        printf("I2C Error: Expected BNO055 CHIP ID 0xa0, received %x\n", bno055_chip_id_val);
    }
    else {
        printf("I2C Info: BNO055 CHIP found.\n");
    }


    while(1) {

        
        vTaskDelay(pdMS_TO_TICKS(500));

        //uint8_t heading_lsb = readByte(BNO055_ADDRESS, BNO055_CHIP_ID_ADDR);
        //printf("heading_lsb %8i\n", heading_lsb);
        

    }
}