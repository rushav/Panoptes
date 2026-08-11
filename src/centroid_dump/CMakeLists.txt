cmake_minimum_required(VERSION 3.10)
project(centroid_dump)

# Check for camera library environment variable
if(NOT DEFINED CAMERA_SDK_PATH)
     message(FATAL_ERROR "ERROR: CAMERA_SDK_PATH is not set! Please provide it when running CMake.")
 endif()

# Find camera library
find_package(CameraLibrary REQUIRED HINTS ${CAMERA_SDK_PATH})

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler
set(SOURCES main.cpp DeviceConnectionManager.cpp KeyPress.cpp)
add_executable(centroid_dump ${SOURCES})

# Linker
if(WIN32)
    target_link_libraries(
        centroid_dump
        CameraLibrary::CameraLibrary
    )
    # Copy CameraLibrary.dll after build
    add_custom_command(
        TARGET centroid_dump POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CAMERA_SDK_PATH}/bin/CameraLibrary.dll"
            "$<TARGET_FILE_DIR:centroid_dump>"
    )
elseif(UNIX)
    target_link_libraries(centroid_dump
        CameraLibrary::CameraLibrary
        rt stdc++ pthread jpeg
    )
endif()

