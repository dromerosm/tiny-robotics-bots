idf_component_register(SRCS "main.c" "atom.c" "plugs.c" "bridge.c" "info.c" "energy.c" "api.c" "json.c" "cloud.c"
                       EMBED_TXTFILES "panel.html"
                       PRIV_REQUIRES tr_ota tr_tuya esp_hap_core esp_hap_platform esp_hap_apple_profiles
                                     mdns esp_http_server esp_app_format esp_timer esp_wifi esp_netif nvs_flash lwip qrcode
                                     esp_driver_rmt esp_driver_gpio esp_http_client mbedtls)
