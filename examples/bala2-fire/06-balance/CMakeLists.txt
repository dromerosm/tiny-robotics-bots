cmake_minimum_required(VERSION 3.16)
# The shared components (bala2_link) live in the repo's top-level components/ folder.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../components")
# Build only what this firmware uses (main and what it requires), not every
# shared component: a shorter build, and StackChan's modules stay out.
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(balance)
