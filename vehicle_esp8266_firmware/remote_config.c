#define CONFIG_GENERATE_VARIABLES 1
#include "remote_config.h"
#include "remote_debug.h"
#include "domains.h"

#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lwip/api.h"
#include <string.h>

static config_variable* find_variable(char* message, size_t offset) {
    const int n_variables = sizeof(config_variables) / sizeof(*config_variables);
    for (int i = 0; i < n_variables; ++i) {
        char* match = strstr(message + offset, config_variables[i].name);
        if(match == message + offset) {
            char next_char_after_name = *(match + strlen(config_variables[i].name));
            if(next_char_after_name == ' ' || next_char_after_name == 0) {
                return (config_variables + i);
            }
        }
    }
    return NULL;
}

static void decode_getset_message(char* message, size_t offset, bool is_get)  {
    config_variable* variable = find_variable(message, offset);
    if(variable != NULL) {
        char* value_str = message + offset + strlen(variable->name);
        switch(variable->type) {
            case CONFIG_TYPE_INT:
                if(!is_get) { variable->value_int = atoi(value_str); }
                remote_debug_printf("set %s %i\n", variable->name, variable->value_int);
            break;

            case CONFIG_TYPE_BOOL:
                if(!is_get) { variable->value_bool = (strstr(value_str, "true") != NULL); }
                remote_debug_printf("set %s %s\n", variable->name, ((variable->value_bool)?("true"):("false")));
            break;

            case CONFIG_TYPE_FLOAT:
                if(!is_get) { variable->value_float = atof(value_str); }
                remote_debug_printf("set %s %e\n", variable->name, variable->value_float);
            break;

            default: break;
        }
    } else {
        remote_debug_printf("Unknown variable name: %s\n", message);  
    }
}


static void decode_config_message(char* message) {
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
            printf("%s: Waiting for WiFi+IP...\n", __FUNCTION__);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        while(get_master_address() == NULL) {
            printf("%s: Waiting master mDNS IP...\n", __FUNCTION__);
            vTaskDelay(pdMS_TO_TICKS(1000));
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
            if(err == ERR_OK && netbuf_len(netbuf) < 256 && ip_addr_cmp(get_master_address(), &(netbuf->addr))) {
                char message[256];
                netbuf_copy(netbuf, message, netbuf_len(netbuf));
                message[netbuf_len(netbuf)] = 0;

                decode_config_message(message);
            }
            netbuf_delete(netbuf);
        }
    }
}