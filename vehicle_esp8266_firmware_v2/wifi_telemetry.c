#include "wifi_telemetry.h"
#include "espressif/esp_common.h"
#include "esp/uart.h"
#include "FreeRTOS.h"
#include "ssid_config.h"
#include "lwip/api.h"
#include <string.h>
#include "master_ip.h"

static struct netconn* conn;

void wifi_telemetry_setup() {


    // UDP socket
    err_t err;
    conn = netconn_new(NETCONN_UDP);
    err = netconn_bind(conn, IP_ADDR_ANY, 6780);
    if (err != ERR_OK) {
        netconn_delete(conn);
        printf("%s : Could not bind! (%s)\n", __FUNCTION__, lwip_strerr(err));
    }
    else {
        printf("Got a socket!\n");
    }

}


void wifi_telemetry_send(void* data_buffer, int n_bytes) {

    ip_addr_t master_address;
    ipaddr_aton(MASTER_IP, &master_address);

    err_t err;
    struct netbuf* buf = NULL;
    void* netbuf_data = NULL;
    buf = netbuf_new();
    netbuf_data = netbuf_alloc(buf, n_bytes);
    memcpy (netbuf_data, data_buffer, n_bytes);
    err = netconn_sendto(conn, buf, &master_address, 6780);

    if (err != ERR_OK) {
        printf("%s : Could not send data! (%s)\n", __FUNCTION__, lwip_strerr(err));
        netconn_delete(conn);
    }
    netbuf_delete(buf);
}