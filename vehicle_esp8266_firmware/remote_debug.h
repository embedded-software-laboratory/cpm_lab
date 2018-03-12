#pragma once

#include <stdarg.h>

// Must be called once, before using remote_debug_printf().
void init_remote_debug();

// Same usage as printf(), but places the resulting string in a UDP packet queue, which is broadcast via WiFi on port 6780.
// The resulting string is truncated after 255 characters.
void remote_debug_printf(const char* format, ...);

void task_remote_debug_sender(void *pvParameters);