idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES tr_ota tr_board esp_http_server
                       EMBED_TXTFILES "app.html")
