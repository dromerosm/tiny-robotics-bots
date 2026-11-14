cmake_minimum_required(VERSION 3.16)
# Shared components (tr_ota, tr_board, tr_web, tr_tuya) live in the repo's top-level components/ folder.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../components")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(homekit)

# Espressif's HomeKit SDK is written for ESP-IDF 5. Three things make it build
# on ESP-IDF 6 without changing its files:
# - the old Wi-Fi interface names;
# - C17, because one of its prototypes with "()" is not valid C23;
# - idf6/mbedtls/*.h, which point to where mbedTLS 4 keeps AES and SHA.
idf_component_get_property(hap_core esp_hap_core COMPONENT_LIB)
target_compile_definitions(${hap_core} PRIVATE ESP_IF_WIFI_STA=WIFI_IF_STA ESP_IF_WIFI_AP=WIFI_IF_AP)
target_compile_options(${hap_core} PRIVATE -std=gnu17)
idf_component_get_property(hap_platform esp_hap_platform COMPONENT_LIB)
target_include_directories(${hap_platform} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/idf6")
