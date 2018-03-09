#pragma once

#ifdef CONFIG_GENERATE_VARIABLES

    const int n_variables = 6;
    config_variable config_variables[] = {
        { .type = CONFIG_TYPE_INT, .name = "intA", .value_int = 21 },
        { .type = CONFIG_TYPE_INT, .name = "intB", .value_int = 42 },
        { .type = CONFIG_TYPE_BOOL, .name = "boolC", .value_bool = true },
        { .type = CONFIG_TYPE_BOOL, .name = "boolD", .value_bool = false },
        { .type = CONFIG_TYPE_FLOAT, .name = "floatE", .value_float = 3.1415927 },
        { .type = CONFIG_TYPE_FLOAT, .name = "floatF", .value_float = -2.7182818 },
    };

#endif

#define CONFIG_VAR_intA   (config_variables[0].value_int)
#define CONFIG_VAR_intB   (config_variables[1].value_int)
#define CONFIG_VAR_boolC  (config_variables[2].value_bool)
#define CONFIG_VAR_boolD  (config_variables[3].value_bool)
#define CONFIG_VAR_floatE (config_variables[4].value_float)
#define CONFIG_VAR_floatF (config_variables[5].value_float)