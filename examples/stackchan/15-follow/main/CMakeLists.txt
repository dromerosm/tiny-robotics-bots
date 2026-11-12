idf_component_register(SRCS "main.c" "talk.c" "rec.c" "track.c" "radar.c" "face.cpp"
                       PRIV_REQUIRES tr_ota tr_board tr_web esp_http_server esp_http_client esp_timer esp_wifi mbedtls)

# esp-sr tells esp-dl to build only the operations its own models need. The
# face model needs others: add its list (see face-req.yml).
idf_build_get_property(build_components BUILD_COMPONENTS)
if("espressif__esp-dl" IN_LIST build_components)
    idf_component_get_property(espdl_dir espressif__esp-dl COMPONENT_DIR)
    include(${espdl_dir}/cmake/compile_register.cmake)
    esp_dl_compile_add_yml(${COMPONENT_DIR}/face-req.yml)
endif()
