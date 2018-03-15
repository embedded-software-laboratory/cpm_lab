#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "domains.h"
#include "lwip/api.h"
#include "lwip/apps/mdns.h"
#include "lwip/ip_addr.h"


ip_addr_t master_address;
ip_addr_t* master_address_ptr = NULL;

ip_addr_t* get_master_address() { return master_address_ptr; }

void task_mDNS_setup(void *pvParameters) {

    while(sdk_wifi_station_get_connect_status() != STATION_GOT_IP || netif_default == NULL) {
        printf("%s: Waiting for WiFi+IP...\n", __FUNCTION__);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Setup own mDNS name
    uint32_t id = sdk_system_get_chip_id();
    char hostname[100];
    sprintf(hostname, "esp_%u", id);
    printf("Hostname: %s.local\n", hostname);
    LOCK_TCPIP_CORE();
    mdns_resp_add_netif(netif_default, hostname, 60);
    UNLOCK_TCPIP_CORE();
    vTaskDelay(pdMS_TO_TICKS(1000));

    // find master mDNS IP
    const char * master_name = "cpmmaster.local";
    while(1) {
        err_t err = netconn_gethostbyname(master_name, &master_address);
        if(err == ERR_OK) break;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    printf("%s == %s\n", master_name, ipaddr_ntoa(&master_address));
    master_address_ptr = &master_address;
    vTaskDelay(pdMS_TO_TICKS(1000));    
    vTaskDelete(NULL);
}
