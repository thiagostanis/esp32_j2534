cmake_minimum_required(VERSION 3.20)
project(esp32_j2534 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

add_library(esp32_j2534 SHARED
    src/j2534.cpp
    src/serial_comm.cpp
)

target_include_directories(esp32_j2534 PRIVATE
    include
    ${CMAKE_SOURCE_DIR}
)

set_target_properties(esp32_j2534 PROPERTIES
    LINK_FLAGS "/DEF:${CMAKE_CURRENT_SOURCE_DIR}/esp32_j2534.def"
)

target_compile_definitions(esp32_j2534 PRIVATE
    WIN32_LEAN_AND_MEAN
    NOMINMAX
)
