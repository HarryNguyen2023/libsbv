#ifndef SBV_WIFI_H
#define SBV_WIFI_H

#include "sbv.h"

#ifdef ESP32xx_IDF
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/sys.h"

#define SBV_WIFI_DEFAULT_SSID               "LGL The Flash"
#define SBV_WIFI_DEFAULT_PASS               "Flash@205"

#define SBV_WIFI_DEFAULT_MAX_CONNECT_TRY    (10)
#define SBV_WIFI_DEFAULT_WAIT_TIME_MS       (5000)

typedef esp_event_base_t                sbv_event_base_t;
typedef wifi_init_config_t              sbv_wifi_init_config_t;
typedef wifi_config_t                   sbv_wifi_configt_t;
typedef esp_event_handler_t             sbv_wifi_event_handler_t;
typedef esp_event_handler_instance_t    sbv_event_handler_instance_t;

typedef enum sbv_wifi_mode_t
{
    SBV_WIFI_MODE_NULL = 0,  /* null mode */
    SBV_WIFI_MODE_STA,       /* WiFi station mode */
    SBV_WIFI_MODE_AP,        /* WiFi soft-AP mode */
    SBV_WIFI_MODE_APSTA      /* WiFi station + soft-AP mode */
} sbv_wifi_mode_t;

typedef enum sbv_wifi_event_bit_t
{
    SBV_WIFI_FAIL_BIT       = BIT0,
    SBV_WIFI_CONNECTED_BIT  = BIT1
} sbv_wifi_event_bit_t;

typedef struct sbv_wifi_instance_t
{
    sbv_event_group_handle_t        sbv_wifi_event_group;
    sbv_wifi_mode_t                 mode;
    uint16_t                        conn_try_num;
    sbv_event_handler_instance_t    wifi_event_instance;
    sbv_event_handler_instance_t    ip_event_instance;
} sbv_wifi_instance_t;

int
sbv_wifi_init(sbv_wifi_mode_t mode);
void
sbv_wifi_sta_deinit(void);
int
sbv_wifi_sta_wait_for_connection(void);

#endif /*ESP32xx_IDF*/
#endif /*SBV_WIFI_H*/