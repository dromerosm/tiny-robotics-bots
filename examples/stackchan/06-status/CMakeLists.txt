cmake_minimum_required(VERSION 3.16)
# Shared components (tr_ota, tr_board) live in the repo's top-level components/ folder.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../components")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(status)
