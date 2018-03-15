#pragma once
#include "lwip/ip_addr.h"

ip_addr_t* get_master_address();
void task_mDNS_setup(void *pvParameters);