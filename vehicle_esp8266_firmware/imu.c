#include "imu.h"


#include <i2c/i2c.h>
#include <string.h>
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"


#define BNO055_ADDRESS_A (0x28)
#define BNO055_ADDRESS_B (0x29)
#define BNO055_ID        (0xA0)

typedef enum
{
    /* Page id register definition */
    BNO055_PAGE_ID_ADDR                                     = 0X07,

    /* PAGE0 REGISTER DEFINITION START*/
    BNO055_CHIP_ID_ADDR                                     = 0x00,
    BNO055_ACCEL_REV_ID_ADDR                                = 0x01,
    BNO055_MAG_REV_ID_ADDR                                  = 0x02,
    BNO055_GYRO_REV_ID_ADDR                                 = 0x03,
    BNO055_SW_REV_ID_LSB_ADDR                               = 0x04,
    BNO055_SW_REV_ID_MSB_ADDR                               = 0x05,
    BNO055_BL_REV_ID_ADDR                                   = 0X06,

    /* Accel data register */
    BNO055_ACCEL_DATA_X_LSB_ADDR                            = 0X08,
    BNO055_ACCEL_DATA_X_MSB_ADDR                            = 0X09,
    BNO055_ACCEL_DATA_Y_LSB_ADDR                            = 0X0A,
    BNO055_ACCEL_DATA_Y_MSB_ADDR                            = 0X0B,
    BNO055_ACCEL_DATA_Z_LSB_ADDR                            = 0X0C,
    BNO055_ACCEL_DATA_Z_MSB_ADDR                            = 0X0D,

    /* Mag data register */
    BNO055_MAG_DATA_X_LSB_ADDR                              = 0X0E,
    BNO055_MAG_DATA_X_MSB_ADDR                              = 0X0F,
    BNO055_MAG_DATA_Y_LSB_ADDR                              = 0X10,
    BNO055_MAG_DATA_Y_MSB_ADDR                              = 0X11,
    BNO055_MAG_DATA_Z_LSB_ADDR                              = 0X12,
    BNO055_MAG_DATA_Z_MSB_ADDR                              = 0X13,

    /* Gyro data registers */
    BNO055_GYRO_DATA_X_LSB_ADDR                             = 0X14,
    BNO055_GYRO_DATA_X_MSB_ADDR                             = 0X15,
    BNO055_GYRO_DATA_Y_LSB_ADDR                             = 0X16,
    BNO055_GYRO_DATA_Y_MSB_ADDR                             = 0X17,
    BNO055_GYRO_DATA_Z_LSB_ADDR                             = 0X18,
    BNO055_GYRO_DATA_Z_MSB_ADDR                             = 0X19,

    /* Euler data registers */
    BNO055_EULER_H_LSB_ADDR                                 = 0X1A,
    BNO055_EULER_H_MSB_ADDR                                 = 0X1B,
    BNO055_EULER_R_LSB_ADDR                                 = 0X1C,
    BNO055_EULER_R_MSB_ADDR                                 = 0X1D,
    BNO055_EULER_P_LSB_ADDR                                 = 0X1E,
    BNO055_EULER_P_MSB_ADDR                                 = 0X1F,

    /* Quaternion data registers */
    BNO055_QUATERNION_DATA_W_LSB_ADDR                       = 0X20,
    BNO055_QUATERNION_DATA_W_MSB_ADDR                       = 0X21,
    BNO055_QUATERNION_DATA_X_LSB_ADDR                       = 0X22,
    BNO055_QUATERNION_DATA_X_MSB_ADDR                       = 0X23,
    BNO055_QUATERNION_DATA_Y_LSB_ADDR                       = 0X24,
    BNO055_QUATERNION_DATA_Y_MSB_ADDR                       = 0X25,
    BNO055_QUATERNION_DATA_Z_LSB_ADDR                       = 0X26,
    BNO055_QUATERNION_DATA_Z_MSB_ADDR                       = 0X27,

    /* Linear acceleration data registers */
    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR                     = 0X28,
    BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR                     = 0X29,
    BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR                     = 0X2A,
    BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR                     = 0X2B,
    BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR                     = 0X2C,
    BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR                     = 0X2D,

    /* Gravity data registers */
    BNO055_GRAVITY_DATA_X_LSB_ADDR                          = 0X2E,
    BNO055_GRAVITY_DATA_X_MSB_ADDR                          = 0X2F,
    BNO055_GRAVITY_DATA_Y_LSB_ADDR                          = 0X30,
    BNO055_GRAVITY_DATA_Y_MSB_ADDR                          = 0X31,
    BNO055_GRAVITY_DATA_Z_LSB_ADDR                          = 0X32,
    BNO055_GRAVITY_DATA_Z_MSB_ADDR                          = 0X33,

    /* Temperature data register */
    BNO055_TEMP_ADDR                                        = 0X34,

    /* Status registers */
    BNO055_CALIB_STAT_ADDR                                  = 0X35,
    BNO055_SELFTEST_RESULT_ADDR                             = 0X36,
    BNO055_INTR_STAT_ADDR                                   = 0X37,

    BNO055_SYS_CLK_STAT_ADDR                                = 0X38,
    BNO055_SYS_STAT_ADDR                                    = 0X39,
    BNO055_SYS_ERR_ADDR                                     = 0X3A,

    /* Unit selection register */
    BNO055_UNIT_SEL_ADDR                                    = 0X3B,
    BNO055_DATA_SELECT_ADDR                                 = 0X3C,

    /* Mode registers */
    BNO055_OPR_MODE_ADDR                                    = 0X3D,
    BNO055_PWR_MODE_ADDR                                    = 0X3E,

    BNO055_SYS_TRIGGER_ADDR                                 = 0X3F,
    BNO055_TEMP_SOURCE_ADDR                                 = 0X40,

    /* Axis remap registers */
    BNO055_AXIS_MAP_CONFIG_ADDR                             = 0X41,
    BNO055_AXIS_MAP_SIGN_ADDR                               = 0X42,

    /* SIC registers */
    BNO055_SIC_MATRIX_0_LSB_ADDR                            = 0X43,
    BNO055_SIC_MATRIX_0_MSB_ADDR                            = 0X44,
    BNO055_SIC_MATRIX_1_LSB_ADDR                            = 0X45,
    BNO055_SIC_MATRIX_1_MSB_ADDR                            = 0X46,
    BNO055_SIC_MATRIX_2_LSB_ADDR                            = 0X47,
    BNO055_SIC_MATRIX_2_MSB_ADDR                            = 0X48,
    BNO055_SIC_MATRIX_3_LSB_ADDR                            = 0X49,
    BNO055_SIC_MATRIX_3_MSB_ADDR                            = 0X4A,
    BNO055_SIC_MATRIX_4_LSB_ADDR                            = 0X4B,
    BNO055_SIC_MATRIX_4_MSB_ADDR                            = 0X4C,
    BNO055_SIC_MATRIX_5_LSB_ADDR                            = 0X4D,
    BNO055_SIC_MATRIX_5_MSB_ADDR                            = 0X4E,
    BNO055_SIC_MATRIX_6_LSB_ADDR                            = 0X4F,
    BNO055_SIC_MATRIX_6_MSB_ADDR                            = 0X50,
    BNO055_SIC_MATRIX_7_LSB_ADDR                            = 0X51,
    BNO055_SIC_MATRIX_7_MSB_ADDR                            = 0X52,
    BNO055_SIC_MATRIX_8_LSB_ADDR                            = 0X53,
    BNO055_SIC_MATRIX_8_MSB_ADDR                            = 0X54,

    /* Accelerometer Offset registers */
    ACCEL_OFFSET_X_LSB_ADDR                                 = 0X55,
    ACCEL_OFFSET_X_MSB_ADDR                                 = 0X56,
    ACCEL_OFFSET_Y_LSB_ADDR                                 = 0X57,
    ACCEL_OFFSET_Y_MSB_ADDR                                 = 0X58,
    ACCEL_OFFSET_Z_LSB_ADDR                                 = 0X59,
    ACCEL_OFFSET_Z_MSB_ADDR                                 = 0X5A,

    /* Magnetometer Offset registers */
    MAG_OFFSET_X_LSB_ADDR                                   = 0X5B,
    MAG_OFFSET_X_MSB_ADDR                                   = 0X5C,
    MAG_OFFSET_Y_LSB_ADDR                                   = 0X5D,
    MAG_OFFSET_Y_MSB_ADDR                                   = 0X5E,
    MAG_OFFSET_Z_LSB_ADDR                                   = 0X5F,
    MAG_OFFSET_Z_MSB_ADDR                                   = 0X60,

    /* Gyroscope Offset register s*/
    GYRO_OFFSET_X_LSB_ADDR                                  = 0X61,
    GYRO_OFFSET_X_MSB_ADDR                                  = 0X62,
    GYRO_OFFSET_Y_LSB_ADDR                                  = 0X63,
    GYRO_OFFSET_Y_MSB_ADDR                                  = 0X64,
    GYRO_OFFSET_Z_LSB_ADDR                                  = 0X65,
    GYRO_OFFSET_Z_MSB_ADDR                                  = 0X66,

    /* Radius registers */
    ACCEL_RADIUS_LSB_ADDR                                   = 0X67,
    ACCEL_RADIUS_MSB_ADDR                                   = 0X68,
    MAG_RADIUS_LSB_ADDR                                     = 0X69,
    MAG_RADIUS_MSB_ADDR                                     = 0X6A
} bno055_reg_t;

typedef enum
{
  POWER_MODE_NORMAL                                       = 0X00,
  POWER_MODE_LOWPOWER                                     = 0X01,
  POWER_MODE_SUSPEND                                      = 0X02
} bno055_powermode_t;

typedef enum
{
    /* Operation mode settings*/
    OPERATION_MODE_CONFIG                                   = 0X00,
    OPERATION_MODE_ACCONLY                                  = 0X01,
    OPERATION_MODE_MAGONLY                                  = 0X02,
    OPERATION_MODE_GYRONLY                                  = 0X03,
    OPERATION_MODE_ACCMAG                                   = 0X04,
    OPERATION_MODE_ACCGYRO                                  = 0X05,
    OPERATION_MODE_MAGGYRO                                  = 0X06,
    OPERATION_MODE_AMG                                      = 0X07,
    OPERATION_MODE_IMUPLUS                                  = 0X08,
    OPERATION_MODE_COMPASS                                  = 0X09,
    OPERATION_MODE_M4G                                      = 0X0A,
    OPERATION_MODE_NDOF_FMC_OFF                             = 0X0B,
    OPERATION_MODE_NDOF                                     = 0X0C
} bno055_opmode_t;

typedef enum
{
    REMAP_CONFIG_P0                                         = 0x21,
    REMAP_CONFIG_P1                                         = 0x24, // default
    REMAP_CONFIG_P2                                         = 0x24,
    REMAP_CONFIG_P3                                         = 0x21,
    REMAP_CONFIG_P4                                         = 0x24,
    REMAP_CONFIG_P5                                         = 0x21,
    REMAP_CONFIG_P6                                         = 0x21,
    REMAP_CONFIG_P7                                         = 0x24
} bno055_axis_remap_config_t;

typedef enum
{
    REMAP_SIGN_P0                                           = 0x04,
    REMAP_SIGN_P1                                           = 0x00, // default
    REMAP_SIGN_P2                                           = 0x06,
    REMAP_SIGN_P3                                           = 0x02,
    REMAP_SIGN_P4                                           = 0x03,
    REMAP_SIGN_P5                                           = 0x01,
    REMAP_SIGN_P6                                           = 0x07,
    REMAP_SIGN_P7                                           = 0x05
} bno055_axis_remap_sign_t;

typedef struct bno055_rev_info_t
{
    uint8_t  accel_rev;
    uint8_t  mag_rev;
    uint8_t  gyro_rev;
    uint16_t sw_rev;
    uint8_t  bl_rev;
} bno055_rev_info_t;

typedef enum
{
    VECTOR_ACCELEROMETER = BNO055_ACCEL_DATA_X_LSB_ADDR,
    VECTOR_MAGNETOMETER  = BNO055_MAG_DATA_X_LSB_ADDR,
    VECTOR_GYROSCOPE     = BNO055_GYRO_DATA_X_LSB_ADDR,
    VECTOR_EULER         = BNO055_EULER_H_LSB_ADDR,
    VECTOR_LINEARACCEL   = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
    VECTOR_GRAVITY       = BNO055_GRAVITY_DATA_X_LSB_ADDR
} vector_type_t;



#define I2C_BUS (0)
#define I2C_ADDR (0x28)
#define BNO055_ID (0xA0)
#define SDA_PIN (2)
#define SCL_PIN (14)


uint16_t write8(uint8_t reg, uint8_t value);
uint16_t read8(uint8_t reg);
uint16_t readLen(uint8_t reg, uint8_t * buffer, uint8_t len);


void init_imu() {

    int init = i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_100K);
    vTaskDelay(pdMS_TO_TICKS(250));
    if(init == -EINVAL){
        printf("%s: invalid bus\n", __FUNCTION__);
    }
    else if(init == -ENOTSUP){
        printf("%s: speed eror\n", __FUNCTION__);
    }

    i2c_set_clock_stretch(I2C_BUS, 80000);

    uint8_t id = read8(BNO055_CHIP_ID_ADDR);
    printf("Found id: %u\n", id);
    if(id != BNO055_ID) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        id = read8(BNO055_CHIP_ID_ADDR);
        if(id != BNO055_ID) {
            printf("%s: Couldn't find the bno055 sensor!\n", __FUNCTION__);
        }
    }

    write8(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    vTaskDelay(pdMS_TO_TICKS(30));


    /* Reset */
    write8(BNO055_SYS_TRIGGER_ADDR, 0x20);
    while (read8(BNO055_CHIP_ID_ADDR) != BNO055_ID)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Set to normal power mode */
    write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    vTaskDelay(pdMS_TO_TICKS(10));

    write8(BNO055_PAGE_ID_ADDR, 0);

    write8(BNO055_SYS_TRIGGER_ADDR, 0x0);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Set the requested operating mode (see section 3.3) */
    write8(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
    vTaskDelay(pdMS_TO_TICKS(20));
}


bool bno055_getVector3(int16_t* vec3, vector_type_t vector_type)
{
    uint8_t buffer[6];
    memset (buffer, 0, sizeof(uint8_t)*6);

    int16_t x, y, z;
    x = y = z = 0;

    /* Read vector data (6 bytes) */
    if(readLen((bno055_reg_t)vector_type, buffer, 6) > 0)
        return false;
    x = ((int16_t)buffer[0]) | (((int16_t)buffer[1]) << 8);
    y = ((int16_t)buffer[2]) | (((int16_t)buffer[3]) << 8);
    z = ((int16_t)buffer[4]) | (((int16_t)buffer[5]) << 8);

    vec3[0] = x;
    vec3[1] = y;
    vec3[2] = z;

    return true;
}

uint16_t write8(uint8_t reg, uint8_t value) {
    return i2c_slave_write(I2C_BUS, I2C_ADDR, &reg, &value, 1) << 8;
}

uint16_t read8(uint8_t reg){
    uint8_t res;
    uint16_t ret = i2c_slave_read(I2C_BUS, I2C_ADDR, &reg, &res, 1);
    if (ret){
        uint16_t error = ((uint16_t)ret << 8);
        return error;
    }
    return res;
}

uint16_t readLen(uint8_t reg, uint8_t* buffer, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++)
    {
        uint8_t res;
        uint16_t ret = i2c_slave_read(I2C_BUS, I2C_ADDR, &reg, &res, 1);
        if (ret){
            uint16_t error = ((uint16_t)ret << 8);
            return error;
        }
        buffer[i] = res;
        reg += sizeof(reg);
    }
    return 0;
}


bool imu_get_yaw(float* yaw_radians) {
    int16_t vec[3];
    if(bno055_getVector3(vec, VECTOR_EULER)) {
        *yaw_radians = ((float)(vec[0] - 2880)) * (-0.00109083078249645598);
        return true;
    }
    return false;
}