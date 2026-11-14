idf_component_register(SRCS "main.c" "plugs.c" "bridge.c"
                       PRIV_REQUIRES tr_ota tr_board tr_web tr_tuya esp_hap_core esp_hap_apple_profiles
                                     mdns esp_http_server esp_timer esp_wifi esp_netif nvs_flash lwip qrcode)
