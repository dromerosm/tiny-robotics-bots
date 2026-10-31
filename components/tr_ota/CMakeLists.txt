idf_component_register(SRCS "tr_ota.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_wifi esp_netif esp_event nvs_flash
                                     esp_http_server app_update esp_app_format esp_timer)
