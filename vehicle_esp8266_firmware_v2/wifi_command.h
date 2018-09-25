#pragma once
#include <stdbool.h>
#include "spi_atmega.h"


void task_wifi_command(void *pvParameters);
bool wifi_get_command(spi_mosi_data_t* out);