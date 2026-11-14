cmake_minimum_required(VERSION 3.16)
project(sync CXX)

set(CMAKE_CXX_STANDARD 17)
set(SDK_DIR "$ENV{HOME}/optitrack/CameraSDK")

find_package(OpenCV REQUIRED)

add_executable(sync main.cpp
    ../../lib/camera.cpp
    ../../lib/maskfile.cpp
    ../../lib/sync.cpp)
target_include_directories(sync PRIVATE ${SDK_DIR}/include)
target_link_directories(sync PRIVATE ${SDK_DIR}/lib)
target_link_libraries(sync PRIVATE CameraLibrary jpeg ${OpenCV_LIBS})
