idf_component_register(SRCS "tr_board.c" "tr_lcd.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c
                       PRIV_REQUIRES esp_driver_spi esp_lcd)
