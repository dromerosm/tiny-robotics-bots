# StackChan only (CoreS3, ESP32-S3). For other chips (BALA2's ESP32) the
# component is empty, so the examples there still build.
if(NOT IDF_TARGET STREQUAL "esp32s3")
    idf_component_register()
    return()
endif()

idf_component_register(SRCS "tr_board.c" "tr_lcd.c" "tr_servo.c" "tr_sensors.c" "tr_mic.c" "tr_audio.c" "tr_cam.c"
                            "tr_sleep.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_driver_i2c
                       PRIV_REQUIRES esp_driver_spi esp_driver_uart esp_driver_tsens esp_driver_i2s
                                     esp_lcd esp_timer esp_pm esp_driver_gpio nvs_flash)
