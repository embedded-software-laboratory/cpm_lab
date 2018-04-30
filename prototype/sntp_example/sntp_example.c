/*
 * Test code for SNTP on esp-open-rtos.
 *
 * Jesus Alonso (doragasu)
 */
#include <espressif/esp_common.h>
#include <esp/uart.h>

#include <string.h>
#include <stdio.h>

#include <FreeRTOS.h>
#include <task.h>

#include <lwip/err.h>
#include <lwip/sockets.h>
#include <lwip/sys.h>
#include <lwip/netdb.h>
#include <lwip/dns.h>

#include <ssid_config.h>

/* Add extras/sntp component to makefile for this include to work */
#include <sntp.h>
#include <time.h>

#define SNTP_SERVERS     "192.168.0.118", "192.168.0.118", \
                        "192.168.0.118", "3.pool.ntp.org"

#define vTaskDelayMs(ms)    vTaskDelay((ms)/portTICK_PERIOD_MS)
#define UNUSED_ARG(x)    (void)x

void sntp_tsk(void *pvParameters)
{
    char *servers[] = {SNTP_SERVERS};
    UNUSED_ARG(pvParameters);

    /* Wait until we have joined AP and are assigned an IP */
    while (sdk_wifi_station_get_connect_status() != STATION_GOT_IP) {
        vTaskDelayMs(100);
    }

    /* Start SNTP */
    printf("Starting SNTP... ");
    LOCK_TCPIP_CORE();
    /* SNTP will request an update each 5 minutes */
    sntp_set_update_delay(15000);
    /* Set GMT+1 zone, daylight savings off */
    const struct timezone tz = {1*60, 0};
    /* SNTP initialization */
    sntp_initialize(&tz);
    /* Servers must be configured right after initialization */
    sntp_set_servers(servers, sizeof(servers) / sizeof(char*));
    UNLOCK_TCPIP_CORE();
    printf("DONE!\n");

    while(1) {
        int32_t us;
        time_t ts2 = sntp_get_rtc_time(&us);
        int32_t secs = ts2;
        //printf("sec: %i.%06i\n", secs, us);

        gpio_write(5, secs%2);
        vTaskDelayMs(1);
    }
}

void user_init(void)
{
    uart_set_baud(0, 115200);
    printf("SDK version:%s\n", sdk_system_get_sdk_version());

    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };
    gpio_enable(5, GPIO_OUTPUT);
    gpio_write(5, 0);

    /* required to call wifi_set_opmode before station_set_config */
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);

    xTaskCreate(sntp_tsk, "SNTP", 1024, NULL, 1, NULL);
}

