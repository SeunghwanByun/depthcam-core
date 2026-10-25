cmake_minimum_required(VERSION 3.16)

project(ProjectYouth)

if(WIN32)
    message("Building on Windows")
elseif(UNIX AND NOT APPLE)
    message("Building on Linux")
elseif(APPLE)
    message("Building on macOS")
endif()

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Sub directory addition
add_subdirectory(LoggingModule)
add_subdirectory(SensorModule)
add_subdirectory(ZmqPublisher)
add_subdirectory(AlgorithmModule)
add_subdirectory(ViewerModule)

# Find Library
# OpenGL
find_package(OpenGL REQUIRED)
find_library(GLU_LIB GLU REQUIRED)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)

# Generate execution file
add_executable(Youth main.c main.h)

# Find link libraries
target_link_libraries(Youth
    LoggingModuleLib
    SensorModuleLib
    ViewerModuleLib
    AlgorithmModuleLib
    ZmqPublisherLib
    OpenGL::GL
    ${GLU_LIB}
    GLEW::GLEW
    GLUT::GLUT
    stdc++
    pthread
    rt
)

# Offline RGB-D mapping without a camera or display.
add_executable(recording_to_map tools/recording_to_map.cpp)
target_link_libraries(recording_to_map AlgorithmModuleLib pthread)

include(CTest)
if(BUILD_TESTING)
    add_executable(test_vo_camera tests/test_vo_camera.cpp)
    target_link_libraries(test_vo_camera AlgorithmModuleLib pthread)
    add_test(NAME vo_camera_snapshot COMMAND test_vo_camera)
endif()
