cmake_minimum_required(VERSION 3.16)
project(detect CXX)

set(CMAKE_CXX_STANDARD 17)
set(SDK_DIR "$ENV{HOME}/optitrack/CameraSDK")

find_package(OpenCV REQUIRED)

add_executable(detect main.cpp)
target_include_directories(detect PRIVATE ${SDK_DIR}/include)
target_link_directories(detect PRIVATE ${SDK_DIR}/lib)
target_link_libraries(detect PRIVATE CameraLibrary jpeg ${OpenCV_LIBS})