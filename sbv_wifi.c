#include <stdio.h>
#include <string.h>

#include "sbv.h"
#include "sbv_rtos.h"
#include "sbv_wifi.h"

#ifdef ESP32xx_IDF
static sbv_wifi_instance_t sbv_wifi_instance;

static int
sbv_wifi_nvs_flash_init(void)
{
    int ret = SBV_OK;

    ret = nvs_flash_init();
    if(ret == ESP_ERR_NVS_NO_FREE_PAGES
       || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ret = nvs_flash_erase();
        if(ret != SBV_OK)
            return SBV_ERROR;

        ret = nvs_flash_init();
        if(ret != SBV_OK)
            return SBV_ERROR;
    }

    return ret;
}

static int
sbv_wifi_default_init(void)
{
    int ret = SBV_OK;
    sbv_wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();

    memset(&sbv_wifi_instance, 0, sizeof (sbv_wifi_instance_t));

    sbv_wifi_instance.sbv_wifi_event_group = sbv_rtos_event_group_create();

    /* Initiate the TCP/IP stack */
    ret = esp_netif_init();
    if(ret != SBV_OK)
        return SBV_ERROR;

    /* Initiate the event loop check */
    ret = esp_event_loop_create_default();
    if(ret != SBV_OK)
        return SBV_ERROR;

    esp_wifi_init(&wifi_cfg);

    return ret;
}

static int
sbv_wifi_sta_connection_init(void)
{
    int ret;
    sbv_wifi_configt_t sta_wifi_config = {
        .sta = {
            .ssid               = SBV_WIFI_DEFAULT_SSID,
            .password           = SBV_WIFI_DEFAULT_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable        = SBV_TRUE,
                .required       = SBV_FALSE
            }
        }
    };

    ret = esp_wifi_set_config(WIFI_IF_STA, &sta_wifi_config);
    if(ret != SBV_OK)
        return SBV_ERROR;

    return SBV_OK;
}


static int
sbv_wifi_connect(void)
{
    return esp_wifi_connect();
}

static void
sbv_wifi_sta_event_handler(void *arg, sbv_event_base_t event_base, int32_t event_id, void *event_data)
{
    /* Case ready to connect */
    if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        sbv_wifi_connect();
    }
    /* Case timeout or connect refused */
    else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        /* In case in 10 consecutives time, the connection to AP is refused, we stop to connect */
        if(sbv_wifi_instance.conn_try_num++ < SBV_WIFI_DEFAULT_MAX_CONNECT_TRY)
        {
            sbv_wifi_connect();
            printf("[WIFI] Retry to connect to AP: time %d\r\n", sbv_wifi_instance.conn_try_num);
        }
        else
        {
            sbv_rtos_event_group_set_bits(sbv_wifi_instance.sbv_wifi_event_group, SBV_WIFI_FAIL_BIT);
            printf("[WIFI] Failed to connect to AP\r\n");
        }
    }
    /* Case got IP address from DHCP server associated with the AP */
    else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* ip_event = (ip_event_got_ip_t *)event_data;
        printf("[WIFI] Got IP: " IPSTR, IP2STR(&ip_event->ip_info.ip));
        sbv_wifi_instance.conn_try_num = 0;
        sbv_rtos_event_group_set_bits(sbv_wifi_instance.sbv_wifi_event_group, SBV_WIFI_CONNECTED_BIT);
    }
}

static int
sbv_wifi_sta_init(sbv_wifi_event_handler_t event_handler, void *arg)
{
    int ret = SBV_OK;

    esp_netif_create_default_wifi_sta();

    ret = esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                              event_handler, arg, &(sbv_wifi_instance.wifi_event_instance));
    if(ret != SBV_OK)
        return SBV_ERROR;

    ret = esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                              event_handler, arg, &(sbv_wifi_instance.ip_event_instance));
    if(ret != SBV_OK)
        return SBV_ERROR;

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if(ret != SBV_OK)
        return SBV_ERROR;

    ret = sbv_wifi_sta_connection_init();
    if(ret != SBV_OK)
        return SBV_ERROR;

    return ret;
}

void
sbv_wifi_sta_deinit(void)
{
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, sbv_wifi_instance.wifi_event_instance);

    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, sbv_wifi_instance.ip_event_instance);

    sbv_rtos_event_group_delete(sbv_wifi_instance.sbv_wifi_event_group);
}

int
sbv_wifi_init(sbv_wifi_mode_t mode)
{
    int ret = SBV_OK;

    ret = sbv_wifi_nvs_flash_init();
    if(ret != SBV_OK)
        return SBV_ERROR;

    ret = sbv_wifi_default_init();
    if(ret != SBV_OK)
        return SBV_ERROR;

    sbv_wifi_instance.mode         = mode;
    sbv_wifi_instance.conn_try_num = 0;

    switch (mode)
    {
        case SBV_WIFI_MODE_STA:
            ret = sbv_wifi_sta_init(sbv_wifi_sta_event_handler, NULL);
            if(ret != SBV_OK)
                return SBV_ERROR;
            break;

        case SBV_WIFI_MODE_AP:
            /* code */
            break;

        case SBV_WIFI_MODE_APSTA:
            break;

        default:
            break;
    }

    ret = esp_wifi_start();
    if(ret != SBV_OK)
        return SBV_ERROR;

    return ret;
}

int
sbv_wifi_sta_wait_for_connection(void)
{
    int ret = SBV_ERROR;
    sbv_rtos_event_bits_t wifi_sta_event_bits;
    sbv_rtos_tick_type_t tick_to_wait = sbv_rtos_ms_to_tick(SBV_WIFI_DEFAULT_WAIT_TIME_MS);

    wifi_sta_event_bits = sbv_rtos_event_group_wait_bits(sbv_wifi_instance.sbv_wifi_event_group,
                                                         SBV_WIFI_CONNECTED_BIT | SBV_WIFI_FAIL_BIT,
                                                         SBV_RTOS_TRUE, SBV_RTOS_FALSE, tick_to_wait);
    if(wifi_sta_event_bits & SBV_WIFI_CONNECTED_BIT)
    {
        printf("[WIFI] Connected successfully to AP %s\n", SBV_WIFI_DEFAULT_SSID);
        ret = SBV_OK;
    }
    else if(wifi_sta_event_bits & SBV_WIFI_FAIL_BIT)
    {
        printf("[WIFI] Failed to connect to AP %s\n", SBV_WIFI_DEFAULT_SSID);
        ret = SBV_ERROR;
    }

    return ret;
}
#endif /*ESP32xx_IDF*/