idf_component_register(SRCS "main.c" "audio.c"
                       PRIV_REQUIRES tr_ota tr_board tr_web esp_http_server esp_timer nvs_flash)
