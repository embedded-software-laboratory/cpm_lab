#pragma once
#include <stdbool.h>

void init_imu();
bool imu_get_yaw(float* yaw_radians);
