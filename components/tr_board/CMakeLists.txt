idf_component_register(SRCS "tr_board.c" "tr_lcd.c" "tr_servo.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c
                       PRIV_REQUIRES esp_driver_spi esp_driver_uart esp_lcd esp_timer nvs_flash)
