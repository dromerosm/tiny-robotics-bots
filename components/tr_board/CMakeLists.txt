idf_component_register(SRCS "tr_board.c" "tr_lcd.c" "tr_servo.c" "tr_sensors.c" "tr_mic.c" "tr_cam.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c
                       PRIV_REQUIRES esp_driver_spi esp_driver_uart esp_driver_tsens esp_driver_i2s
                                     esp_lcd esp_timer nvs_flash)
