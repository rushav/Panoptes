cmake_minimum_required(VERSION 3.16)
project(capture CXX)

set(CMAKE_CXX_STANDARD 17)
set(SDK_DIR "$ENV{HOME}/optitrack/CameraSDK")

find_package(OpenCV REQUIRED)

add_executable(capture
    main.cpp
    ../../lib/camera.cpp
    ../../lib/detect.cpp
    ../../lib/coverage.cpp
    ../../lib/session.cpp)

target_include_directories(capture PRIVATE ${SDK_DIR}/include)
target_link_directories(capture PRIVATE ${SDK_DIR}/lib)
target_link_libraries(capture PRIVATE CameraLibrary jpeg ${OpenCV_LIBS})