cmake_minimum_required(VERSION 3.16)
project(discover CXX)

set(CMAKE_CXX_STANDARD 17)

# Where the OptiTrack SDK lives. Adjust if yours is elsewhere.
set(SDK_DIR "$ENV{HOME}/optitrack/CameraSDK")

add_executable(discover main.cpp)
target_include_directories(discover PRIVATE ${SDK_DIR}/include)
target_link_directories(discover PRIVATE ${SDK_DIR}/lib)
target_link_libraries(discover PRIVATE CameraLibrary jpeg)