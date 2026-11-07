idf_component_register(SRCS "tr_tuya.c" "tr_tuya_esp.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES mbedtls lwip esp_hw_support)
