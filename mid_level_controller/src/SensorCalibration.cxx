// MIT License
// 
// Copyright (c) 2020 Lehrstuhl Informatik 11 - RWTH Aachen University
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
// 
// This file is part of cpm_lab.
// 
// Author: i11 - Embedded Software, RWTH Aachen University

#include "SensorCalibration.hpp"


VehicleState SensorCalibration::convert(spi_miso_data_t spi_miso_data) {
    const double odometer_meter_per_step = 0.0031225604996;
    const double speed_meter_per_second_per_step = odometer_meter_per_step * 0.2384185791;
    const double imu_yaw_radian_per_step = (0.00109083078249645598/50*32);
    const double imu_yaw_rate_radian_per_second_per_step = 0.00109083078249645598;
    const double battery_volt_per_step = 0.0116669;
    const double imu_meter_per_second_per_second_per_step = 0.01;

    // current measurment is non-linear w.r.t to actual current
    // TODO calibration curve
    const double motor_ampere_per_step = 0.01; 


    VehicleState vehicleState;
    vehicleState.odometer_distance           (spi_miso_data.odometer_steps * odometer_meter_per_step);
    vehicleState.pose().x                    (0); // Not measured, TBD by the localization
    vehicleState.pose().y                    (0);
    vehicleState.pose().yaw                  (0);
    vehicleState.imu_acceleration_forward    (spi_miso_data.imu_acceleration_forward * imu_meter_per_second_per_second_per_step);
    vehicleState.imu_acceleration_left       (spi_miso_data.imu_acceleration_left * imu_meter_per_second_per_second_per_step);
    vehicleState.imu_acceleration_up         (spi_miso_data.imu_acceleration_up * imu_meter_per_second_per_second_per_step);
    vehicleState.imu_yaw                     (spi_miso_data.imu_yaw * imu_yaw_radian_per_step);
    vehicleState.imu_yaw_rate                (spi_miso_data.imu_yaw_rate * imu_yaw_rate_radian_per_second_per_step);
    vehicleState.speed                       (spi_miso_data.speed * speed_meter_per_second_per_step);
    vehicleState.battery_voltage             (spi_miso_data.battery_voltage * battery_volt_per_step); //TODO: filter battery voltage 
    vehicleState.motor_current               (spi_miso_data.motor_current * motor_ampere_per_step);
    return vehicleState;
}