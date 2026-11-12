# BALA2 Fire only (ESP32). For other chips (StackChan's ESP32-S3) the
# component is empty, so the examples there still build.
if(NOT IDF_TARGET STREQUAL "esp32")
    idf_component_register()
    return()
endif()

# The icon is the one StackChan's page uses: one file for both robots. It is
# copied under its own name first, because two components cannot embed files
# with the same name in one build (tr_ota embeds favicon.svg too).
configure_file("${CMAKE_CURRENT_LIST_DIR}/../tr_ota/favicon.svg"
               "${CMAKE_CURRENT_BINARY_DIR}/bala2_icon.svg" COPYONLY)

idf_component_register(SRCS "bala2_link.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES esp_wifi esp_netif esp_event nvs_flash esp_http_server
                                     app_update esp_app_format esp_timer
                       EMBED_TXTFILES "${CMAKE_CURRENT_BINARY_DIR}/bala2_icon.svg")
