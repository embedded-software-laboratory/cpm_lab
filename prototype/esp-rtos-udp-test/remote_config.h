#pragma once
#include <stdbool.h>

/*

valid UDP config commands:

set intA -123
get boolB
set floatB 456.123


*/

typedef enum {
    CONFIG_TYPE_INVALID = 0,
    CONFIG_TYPE_INT,
    CONFIG_TYPE_BOOL,
    CONFIG_TYPE_FLOAT,
} config_type_id;

typedef struct {
    config_type_id type;
    char* name;
    union {
        int value_int;
        bool value_bool;
        float value_float;
    };
} config_variable;

extern config_variable config_variables[];
#include "remote_config_variables.h"

void task_remote_config(void *pvParameters);