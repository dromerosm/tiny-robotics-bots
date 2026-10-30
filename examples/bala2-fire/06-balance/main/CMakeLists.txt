idf_component_register(SRCS "main.cpp" "link.cpp"
                       PRIV_REQUIRES esp_driver_i2c esp_driver_gpio esp_driver_uart
                                     esp_wifi esp_netif esp_event nvs_flash
                                     esp_http_server app_update esp_timer)
