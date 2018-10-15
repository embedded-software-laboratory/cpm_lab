#include "SensorCalibration.hpp"


VehicleState SensorCalibration::convert(spi_miso_data_t spi_miso_data) {
    const double odometer_meter_per_step = 0.00468384074941;
    const double speed_meter_per_second_per_step = odometer_meter_per_step * 0.2384185791;
    const double imu_yaw_radian_per_step = -0.00109083078249645598;
    const double battery_volt_per_step = 0.0116669;


    VehicleState vehicleState;
    vehicleState.odometer_distance           (spi_miso_data.odometer_steps * odometer_meter_per_step);
    vehicleState.pose().x                    (0); // Not measured, TBD by the localization
    vehicleState.pose().y                    (0);
    vehicleState.pose().yaw                  (spi_miso_data.imu_yaw * imu_yaw_radian_per_step);
    vehicleState.imu_acceleration_forward    (spi_miso_data.imu_acceleration_forward * 0.01);
    vehicleState.imu_acceleration_left       (spi_miso_data.imu_acceleration_left * 0.01);
    vehicleState.speed                       (spi_miso_data.speed * speed_meter_per_second_per_step);
    vehicleState.battery_voltage             (spi_miso_data.battery_voltage * battery_volt_per_step);
    vehicleState.motor_current               (spi_miso_data.motor_current); // TODO calibrate current
    return vehicleState;
}