cmake_minimum_required(VERSION 3.10)
project(dog_fifo_backend CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES
    daemon/main.cpp
    daemon/motor_io.cpp
    daemon/tcp_server.cpp
    driver/src/robstride.cpp
    driver/src/can_interface.cpp
)

add_executable(dog_fifo_backend ${SOURCES})

find_package(Threads REQUIRED)
target_link_libraries(dog_fifo_backend Threads::Threads)

target_include_directories(dog_fifo_backend PRIVATE
    ${CMAKE_SOURCE_DIR}/daemon
    ${CMAKE_SOURCE_DIR}/driver/include
    ${CMAKE_SOURCE_DIR}/utils/include
)
