cmake_minimum_required(VERSION 3.16)
project(focus CXX)

set(CMAKE_CXX_STANDARD 17)
set(SDK_DIR "$ENV{HOME}/optitrack/CameraSDK")

find_package(OpenCV REQUIRED)

add_executable(focus main.cpp ../../lib/camera.cpp)
target_include_directories(focus PRIVATE ${SDK_DIR}/include)
target_link_directories(focus PRIVATE ${SDK_DIR}/lib)
target_link_libraries(focus PRIVATE CameraLibrary jpeg ${OpenCV_LIBS})