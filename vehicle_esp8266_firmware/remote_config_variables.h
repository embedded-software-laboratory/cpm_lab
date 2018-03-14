#pragma once

#ifdef CONFIG_GENERATE_VARIABLES

    config_variable config_variables[] = {
        //{ .type = CONFIG_TYPE_INT, .name = "intA", .value_int = 21 },
        //{ .type = CONFIG_TYPE_BOOL, .name = "boolC", .value_bool = true },
        //{ .type = CONFIG_TYPE_FLOAT, .name = "floatE", .value_float = 3.1415927 },


        { .type = CONFIG_TYPE_FLOAT, .name = "battery_volts_per_adc_count", .value_float = 0.010356 },
        { .type = CONFIG_TYPE_INT, .name = "motor_signal", .value_int = 1500 },
        { .type = CONFIG_TYPE_FLOAT, .name = "meters_per_odometer_count", .value_float = 0.0078431 },
        { .type = CONFIG_TYPE_FLOAT, .name = "reference_speed", .value_float = 0.0 },
        { .type = CONFIG_TYPE_FLOAT, .name = "speed_error_gain", .value_float = 10.0 },
        { .type = CONFIG_TYPE_FLOAT, .name = "speed_error_integral_gain", .value_float = 1.0 },
    };

#endif

#define CONFIG_VAR_battery_volts_per_adc_count   (config_variables[0].value_float)
#define CONFIG_VAR_motor_signal                  (config_variables[1].value_int)
#define CONFIG_VAR_meters_per_odometer_count     (config_variables[2].value_float)
#define CONFIG_VAR_reference_speed               (config_variables[3].value_float)
#define CONFIG_VAR_speed_error_gain              (config_variables[4].value_float)
#define CONFIG_VAR_speed_error_integral_gain     (config_variables[5].value_float)