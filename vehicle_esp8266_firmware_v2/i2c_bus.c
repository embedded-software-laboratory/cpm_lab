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
#define BNO055_OPR_MODE_ADDR     0X3D
#define BNO055_PWR_MODE_ADDR     0X3E
#define POWER_MODE_NORMAL        0X00
#define OPERATION_MODE_NDOF      0X0C


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

    // I2C Bus setup
    if(i2c_init(0, SCL_PIN, SDA_PIN, I2C_FREQ_100K) != 0) {
        printf("Error in i2c_init()\n");
    }

    i2c_set_clock_stretch(0, 80000);
    vTaskDelay(1);

    // Check for IMU
    uint8_t bno055_chip_id_val = readByte(BNO055_ADDRESS, BNO055_CHIP_ID_ADDR);
    if(bno055_chip_id_val != BNO055_CHIP_ID){
        printf("I2C Error: Expected BNO055 CHIP ID 0xa0, received %x\n", bno055_chip_id_val);
    }
    else {
        printf("I2C Info: BNO055 CHIP found.\n");
    }
    vTaskDelay(1);

    // Configure IMU
    writeByte(BNO055_ADDRESS, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    vTaskDelay(1);

    writeByte(BNO055_ADDRESS, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
    vTaskDelay(1);


    while(1) {
        vTaskDelay(pdMS_TO_TICKS(500));

        uint8_t yaw_data[2];
        uint8_t yaw_register = BNO055_EULER_H_LSB_ADDR;
        if(i2c_slave_read(0, BNO055_ADDRESS, &yaw_register, yaw_data, 2)) {
            printf("Error in i2c_slave_read()\n");
        }

        uint16_t yaw = ((uint16_t)yaw_data[1])<<8 | ((uint16_t)yaw_data[0]);


        printf("yaw %8i\n", yaw);
    }
}