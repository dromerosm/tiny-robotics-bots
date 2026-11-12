# BALA2 Fire only (ESP32). For other chips (StackChan's ESP32-S3) the
# component is empty, so the examples there still build.
if(NOT IDF_TARGET STREQUAL "esp32")
    idf_component_register()
    return()
endif()

idf_component_register(SRCS "bala2_mon.c"
                       INCLUDE_DIRS "include"
                       REQUIRES bala2_link)
