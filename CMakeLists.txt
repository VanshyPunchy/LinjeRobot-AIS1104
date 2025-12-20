cmake_minimum_required(VERSION 3.21)
project(LinjeFølger)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 11)

add_compile_definitions(
        ARDUINO=10819
        ARDUINO_ARCH_ESP32
        F_CPU=240000000L
)

include_directories(
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}/.pio/libdeps/nano_esp32
        ${CMAKE_SOURCE_DIR}/.platformio/packages/framework-arduinoespressif32/cores/esp32
        ${CMAKE_SOURCE_DIR}/.platformio/packages/framework-arduinoespressif32/variants/arduino_nano_esp32
)

file(GLOB_RECURSE SOURCES
        ${CMAKE_SOURCE_DIR}/src/*.cpp
        ${CMAKE_SOURCE_DIR}/src/*.c
        ${CMAKE_SOURCE_DIR}/src/*.h
        ${CMAKE_SOURCE_DIR}/src/*.hpp
        ${CMAKE_SOURCE_DIR}/src/*.ino
)

add_executable(${PROJECT_NAME} ${SOURCES})