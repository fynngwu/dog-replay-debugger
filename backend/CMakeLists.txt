cmake_minimum_required(VERSION 3.10)
project(dog_debug_daemon CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 源文件
set(SOURCES
    daemon/main.cpp
    daemon/twin_agent.cpp
    daemon/motor_io.cpp
    daemon/robstride.cpp
    daemon/can_interface.cpp
)

# 可执行文件
add_executable(dog_debug_daemon ${SOURCES})

# 链接库 (pthread for std::thread)
find_package(Threads REQUIRED)
target_link_libraries(dog_debug_daemon Threads::Threads)

# 包含目录
target_include_directories(dog_debug_daemon PRIVATE ${CMAKE_SOURCE_DIR}/daemon)