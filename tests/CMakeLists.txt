cmake_minimum_required(VERSION 3.16)
project(panoptes_tests CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

# No camera SDK: these lib sources only use camera.h's plain structs.
add_executable(test_autocapture
    test_autocapture.cpp
    ../lib/autocapture.cpp
    ../lib/capture_status.cpp
    ../lib/coverage.cpp
    ../lib/detect.cpp
    ../lib/session.cpp)

target_link_libraries(test_autocapture PRIVATE ${OpenCV_LIBS})
