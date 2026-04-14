cmake_minimum_required(VERSION 3.16)
project(state_machine CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

set(DRIVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../driver)

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
    PATHS /usr/include/x86_64-linux-gnu
)
find_library(NVINFER_LIB nvinfer PATHS /usr/lib/x86_64-linux-gnu)
find_library(NVONNXPARSER_LIB nvonnxparser PATHS /usr/lib/x86_64-linux-gnu)
find_package(CUDA QUIET)

if(TENSORRT_INCLUDE_DIR AND NVINFER_LIB AND NVONNXPARSER_LIB AND CUDA_FOUND)
    set(TENSORRT_FOUND TRUE)
    message(STATUS "TensorRT found")
else()
    set(TENSORRT_FOUND FALSE)
    message(WARNING "TensorRT/CUDA not found, policy mode will not be available")
endif()

set(SM_SOURCES
    main.cpp
    state_machine.cpp
    tcp_server.cpp
    protocol.cpp
    observations.cpp
    serial.c
    wit_c_sdk.c
)

if(TENSORRT_FOUND)
    list(APPEND SM_SOURCES
        tensorrt_inference.cpp
        policy_runner.cpp
    )
endif()

add_executable(dog_sm_server ${SM_SOURCES})

target_include_directories(dog_sm_server PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${DRIVER_DIR}/include
)

if(TENSORRT_FOUND)
    target_include_directories(dog_sm_server PRIVATE
        ${TENSORRT_INCLUDE_DIR}
        ${CUDA_INCLUDE_DIRS}
    )
    target_link_libraries(dog_sm_server PRIVATE
        ${NVINFER_LIB}
        ${NVONNXPARSER_LIB}
        ${CUDA_LIBRARIES}
    )
    target_compile_definitions(dog_sm_server PRIVATE HAS_TENSORRT)
endif()

target_link_libraries(dog_sm_server PRIVATE
    Threads::Threads
    dog_driver
)

target_link_directories(dog_sm_server PRIVATE ${CMAKE_BINARY_DIR}/driver)

target_compile_options(dog_sm_server PRIVATE -Wall -Wextra)

add_executable(dog_client
    tcp_client.cpp
)

target_link_libraries(dog_client Threads::Threads)

target_compile_options(dog_client PRIVATE -Wall -Wextra)

# --- Test CLI tool (links directly to DogDriver, no TCP) ---
add_executable(sm_test test/sm_test.cpp)
target_include_directories(sm_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${DRIVER_DIR}/include
)
target_link_libraries(sm_test PRIVATE Threads::Threads dog_driver)
target_link_directories(sm_test PRIVATE ${CMAKE_BINARY_DIR}/driver)
target_compile_options(sm_test PRIVATE -Wall -Wextra)

add_subdirectory(${DRIVER_DIR} ${CMAKE_BINARY_DIR}/driver)
