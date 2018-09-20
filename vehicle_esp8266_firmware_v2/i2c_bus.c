#include "i2c_bus.h"
#include <i2c/i2c.h>
#include "espressif/esp_common.h"
#include "esp/uart.h"


#define SDA_PIN (2)
#define SCL_PIN (0)

#define BNO055_ADDRESS (0x28)
#define AS5601_ADDRESS (0x36)

#define BNO055_CHIP_ID_ADDR (0x00)
#define BNO055_CHIP_ID      (0xA0)
#define BNO055_EULER_H_LSB_ADDR (0x1A)
#define BNO055_EULER_H_MSB_ADDR (0x1B)
#define BNO055_OPR_MODE_ADDR     0X3D
#define BNO055_PWR_MODE_ADDR     0X3E
#define POWER_MODE_NORMAL        0X00
#define OPERATION_MODE_NDOF      0X0C

#define AS5601_RAW_ANGLE_LSB_ADDR (0x0D)
#define AS5601_RAW_ANGLE_MSB_ADDR (0x0C)


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


#define ODOMETER_BUFFER_SIZE 64
uint8_t led_state = 0; // TODO remove
uint16_t yaw_value = 0;
uint8_t yaw_flag = 0;
int16_t odometer_value = 0;
int16_t odometer_delta_values[ODOMETER_BUFFER_SIZE];
uint8_t odometer_delta_values_index = 0;
uint8_t odometer_flag = 0;


// Interrupt for high frequency I2C communication
// We need to read the odometer at 1kHz (Nyquist frequency)
static void IRAM frc1_interrupt_handler(void *arg) {
    timer_set_load(FRC1, 1562);
    led_state = !led_state;

    gpio_write(5, led_state);

    uint8_t yaw_data[2];
    uint8_t yaw_register = BNO055_EULER_H_LSB_ADDR;
    yaw_flag = i2c_slave_read(0, BNO055_ADDRESS, &yaw_register, yaw_data, 2);
    yaw_value = ((uint16_t)yaw_data[1])<<8 | ((uint16_t)yaw_data[0]);

    sdk_os_delay_us(50);

    uint8_t odometer_data[2];
    uint8_t odometer_register = AS5601_RAW_ANGLE_MSB_ADDR;        
    odometer_flag = i2c_slave_read(0, AS5601_ADDRESS, &odometer_register, odometer_data, 2);
    int16_t odometer_new_value = ((uint16_t)odometer_data[0])<<8 | ((uint16_t)odometer_data[1]);

    odometer_delta_values_index = (odometer_delta_values_index+1) % ODOMETER_BUFFER_SIZE;
    odometer_delta_values[odometer_delta_values_index] = odometer_new_value - odometer_value;
    odometer_value = odometer_new_value;

}


void task_i2c_bus(void *pvParameters) {

    vTaskDelay(pdMS_TO_TICKS(500));

    // I2C Bus setup
    if(i2c_init(0, SCL_PIN, SDA_PIN, I2C_FREQ_400K) != 0) {
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


    // Setup interrupt for high frequency I2C communication
    _xt_isr_attach(INUM_TIMER_FRC1, frc1_interrupt_handler, NULL);
    timer_set_divider(FRC1, TIMER_CLKDIV_256);
    timer_set_load(FRC1, 1);
    timer_set_reload(FRC1, false);
    timer_set_interrupts(FRC1, true);
    timer_set_run(FRC1, true);

    gpio_enable(5, GPIO_OUTPUT);



    while(1) {
        vTaskDelay(pdMS_TO_TICKS(500));


        printf("yaw %6i yawflg %6i odom %6i odomflg %6i\n",
                yaw_value, yaw_flag, odometer_value, odometer_flag);


        int16_t delta_max_abs = 0;
        for (int i = 0; i < ODOMETER_BUFFER_SIZE; ++i)
        {
            int16_t delta = odometer_delta_values[i];

            if(delta > 2048) delta -= 4096;
            else if(delta < -2048) delta += 4096;


            if(delta < 0) delta = -delta;

            if(delta > delta_max_abs) {
                delta_max_abs = delta;
            }
        }

        printf("delta_max_abs %6i\n", delta_max_abs);


/*
        uint8_t yaw_data[2];
        uint8_t yaw_register = BNO055_EULER_H_LSB_ADDR;
        if(i2c_slave_read(0, BNO055_ADDRESS, &yaw_register, yaw_data, 2)) {
            printf("Error in i2c_slave_read()\n");
        }

        uint16_t yaw = ((uint16_t)yaw_data[1])<<8 | ((uint16_t)yaw_data[0]);


        printf("yaw %8i\n", yaw);



        vTaskDelay(pdMS_TO_TICKS(50));
        uint8_t odometer_data[2];
        uint8_t odometer_register = AS5601_RAW_ANGLE_MSB_ADDR;        
        if(i2c_slave_read(0, AS5601_ADDRESS, &odometer_register, odometer_data, 2)) {
            printf("Error in i2c_slave_read()\n");
        }

        uint16_t odometer = ((uint16_t)odometer_data[0])<<8 | ((uint16_t)odometer_data[1]);

        
        printf("odometer %8i\n", odometer);
        */

    }
}