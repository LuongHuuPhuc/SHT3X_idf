cmake_minimum_required(VERSION 3.30)

set(IDF_TARGET esp32)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/sht3x_idf
                         ${CMAKE_CURRENT_SOURCE_DIR}/../Mysource/lsm6ds3_test/components)
                         
project(SHT3x_driver)