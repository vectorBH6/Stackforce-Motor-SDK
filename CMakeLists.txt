cmake_minimum_required(VERSION 3.10)
project(sfmotor_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(sfmotor_control
    src/main.cpp
    include/CAN_comm.cpp
    include/CAN_twai.cpp
)

target_include_directories(sfmotor_control
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

