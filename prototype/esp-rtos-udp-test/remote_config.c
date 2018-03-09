#include "remote_config.h"
#include "remote_debug.h"

#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lwip/api.h"
#include <string.h>
#include <stdbool.h>

/*

valid config commands:

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

const int n_int_variables = 2;
const char* int_variable_names[] = { "intA", "intB" };
int int_variable_values[] = { 21, 42 };


const int n_bool_variables = 2;
const char* bool_variable_names[] = { "boolA", "boolB" };
bool bool_variable_values[] = { true, false };


const int n_float_variables = 2;
const char* float_variable_names[] = { "floatA", "floatB" };
float float_variable_values[] = { 3.1415, 2.7182818 };



void find_variable_index_and_type(char* message, size_t offset, int* out_variable_index, config_type_id* out_variable_type) {
    *out_variable_index = -1;

    for (int i = 0; i < n_int_variables; ++i) {
        char* match = strstr(message + offset, int_variable_names[i]);
        if(match == message + offset) {
            char next_char_after_name = *(match + strlen(int_variable_names[i]));
            if(next_char_after_name == ' ' || next_char_after_name == 0) {
                *out_variable_index = i;
                *out_variable_type = CONFIG_TYPE_INT;
                return;
            }
        }
    }

    for (int i = 0; i < n_bool_variables; ++i) {
        char* match = strstr(message + offset, bool_variable_names[i]);
        if(match == message + offset) {
            char next_char_after_name = *(match + strlen(bool_variable_names[i]));
            if(next_char_after_name == ' ' || next_char_after_name == 0) {
                *out_variable_index = i;
                *out_variable_type = CONFIG_TYPE_BOOL;
                return;
            }
        }
    }

    for (int i = 0; i < n_float_variables; ++i) {
        char* match = strstr(message + offset, float_variable_names[i]);
        if(match == message + offset) {
            char next_char_after_name = *(match + strlen(float_variable_names[i]));
            if(next_char_after_name == ' ' || next_char_after_name == 0) {
                *out_variable_index = i;
                *out_variable_type = CONFIG_TYPE_FLOAT;
                return;
            }
        }
    }
}

void decode_getset_message(char* message, size_t offset, bool is_get)  {
    int variable_index = -1;
    config_type_id variable_type = CONFIG_TYPE_INVALID;
    find_variable_index_and_type(message, offset, &variable_index, &variable_type);

    if(variable_index >= 0 && variable_type != CONFIG_TYPE_INVALID) {
        switch(variable_type) {

            case CONFIG_TYPE_INT:
                if(!is_get) {
                    int_variable_values[variable_index] 
                        = atoi(message + offset + strlen(int_variable_names[variable_index]));
                }
                remote_debug_printf("set %s %i\n", int_variable_names[variable_index], int_variable_values[variable_index]);
            break;

            case CONFIG_TYPE_BOOL:
                if(!is_get) {
                    bool_variable_values[variable_index] 
                        = (strstr(message + offset + strlen(bool_variable_names[variable_index]), "true") != NULL);
                }
                if(bool_variable_values[variable_index]){
                    remote_debug_printf("set %s true\n", bool_variable_names[variable_index]);
                } else {
                    remote_debug_printf("set %s false\n", bool_variable_names[variable_index]);
                }
            break;

            case CONFIG_TYPE_FLOAT:
                if(!is_get) {
                    float_variable_values[variable_index] 
                        = atof(message + offset + strlen(float_variable_names[variable_index]));
                }
                remote_debug_printf("set %s %f\n", float_variable_names[variable_index], float_variable_values[variable_index]);
            break;

            default: break;
        }
    } else {
        remote_debug_printf("Unknown variable name: %s\n", message);  
    }
}


void decode_config_message(char* message) {
    if(message == strstr(message, "get ")) {
        decode_getset_message(message, 4, true);
    } else if(message == strstr(message, "set ")) {
        decode_getset_message(message, 4, false);
    } else {
        remote_debug_printf("Unknown message format: %s\n", message);
    }
}


void task_remote_config(void *pvParameters) {
    while(1) {
        // wait for wifi connection
        while(sdk_wifi_station_get_connect_status() != STATION_GOT_IP) {
            printf("Waiting for WiFi+IP...\n");
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        // UDP socket
        err_t err;
        struct netconn* conn = netconn_new(NETCONN_UDP);
        err = netconn_bind(conn, IP_ADDR_ANY, 6781);
        if (err != ERR_OK) {
            netconn_delete(conn);
            printf("%s : Could not bind! (%s)\n", __FUNCTION__, lwip_strerr(err));
            continue;
        }

        while(1) {

            // Receive message
            struct netbuf *netbuf;
            err = netconn_recv(conn, &netbuf);
            if(err == ERR_OK && netbuf_len(netbuf) < 256) {
                char message[256];
                netbuf_copy(netbuf, message, netbuf_len(netbuf));
                message[netbuf_len(netbuf)] = 0;

                decode_config_message(message);
            }
            netbuf_delete(netbuf);
        }
    }
}