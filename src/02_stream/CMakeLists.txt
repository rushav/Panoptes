cmake_minimum_required(VERSION 3.16)
project(stream CXX)

set(CMAKE_CXX_STANDARD 17)
set(SDK_DIR "$ENV{HOME}/optitrack/CameraSDK")

find_package(OpenCV REQUIRED)

add_executable(stream main.cpp)
target_include_directories(stream PRIVATE ${SDK_DIR}/include)
target_link_directories(stream PRIVATE ${SDK_DIR}/lib)
target_link_libraries(stream PRIVATE CameraLibrary jpeg ${OpenCV_LIBS})