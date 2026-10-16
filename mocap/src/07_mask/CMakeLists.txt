cmake_minimum_required(VERSION 3.16)
project(mask CXX)

set(CMAKE_CXX_STANDARD 17)
set(SDK_DIR "$ENV{HOME}/optitrack/CameraSDK")

find_package(OpenCV REQUIRED)

add_executable(mask main.cpp ../../lib/camera.cpp)
target_include_directories(mask PRIVATE ${SDK_DIR}/include)
target_link_directories(mask PRIVATE ${SDK_DIR}/lib)
target_link_libraries(mask PRIVATE CameraLibrary jpeg ${OpenCV_LIBS})
target_compile_definitions(mask PRIVATE
    MOCAP_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../../config")
