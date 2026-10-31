# StackChan only (CoreS3, ESP32-S3). For other chips (BALA2's ESP32) the
# component is empty, so the examples there still build.
if(NOT IDF_TARGET STREQUAL "esp32s3")
    idf_component_register()
    return()
endif()

idf_component_register(SRCS "tr_web.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES tr_ota tr_board esp_http_server esp_timer
                       EMBED_TXTFILES "app.html")
