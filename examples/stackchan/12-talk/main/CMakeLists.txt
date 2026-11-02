idf_component_register(SRCS "main.c" "talk.c"
                       PRIV_REQUIRES tr_ota tr_board tr_web esp_http_server esp_http_client esp_timer mbedtls)
