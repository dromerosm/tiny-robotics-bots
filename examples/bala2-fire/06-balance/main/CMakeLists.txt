idf_component_register(SRCS "main.cpp" "tof.c"
                       PRIV_REQUIRES esp_driver_i2c esp_driver_gpio esp_driver_uart
                                     bala2_link bala2_mon nvs_flash app_update esp_timer
                       EMBED_TXTFILES "app.html")
