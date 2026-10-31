idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES tr_ota tr_board esp_wifi esp_netif app_update esp_app_format
                                     esp_timer)
