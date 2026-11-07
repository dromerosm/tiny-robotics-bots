idf_component_register(SRCS "main.c" "talk.c" "home.c"
                       PRIV_REQUIRES tr_ota tr_board tr_web tr_tuya esp_http_server esp_http_client esp_timer esp_wifi esp_netif mbedtls)
