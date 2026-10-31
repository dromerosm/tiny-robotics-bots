idf_component_register(SRCS "tr_web.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES tr_ota tr_board esp_http_server esp_timer
                       EMBED_TXTFILES "app.html")
