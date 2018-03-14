#pragma once

#ifdef CONFIG_GENERATE_VARIABLES

    config_variable config_variables[] = {
        //{ .type = CONFIG_TYPE_INT, .name = "intA", .value_int = 21 },
        //{ .type = CONFIG_TYPE_BOOL, .name = "boolC", .value_bool = true },
        //{ .type = CONFIG_TYPE_FLOAT, .name = "floatE", .value_float = 3.1415927 },


        { .type = CONFIG_TYPE_FLOAT, .name = "battery_volts_per_adc_count", .value_float = 0.010356 },
        { .type = CONFIG_TYPE_INT, .name = "motor_signal", .value_int = 1500 },
    };

#endif

#define CONFIG_VAR_battery_volts_per_adc_count   (config_variables[0].value_float)
#define CONFIG_VAR_motor_signal                  (config_variables[1].value_int)