#pragma once
#include <stdbool.h>

bool get_speed_and_curvature_command(float* command_speed_out, float* command_curvature_out);
void task_remote_command(void *pvParameters);