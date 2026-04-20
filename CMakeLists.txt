cmake_minimum_required(VERSION 3.16)

project(ProjectYouth)

option(YOUTH_ENABLE_ALGORITHM "Build the ORB_SLAM3-based algorithm module" OFF)

# Set Debugging Symbols
# set(CMAKE_BUILD_TYPE Debug)

if(WIN32)
    message("Building on Windows")
    # Windows-specific settings
elseif(UNIX AND NOT APPLE)
    message("Building on Linux")
elseif(APPLE)
    message("Building on macOS")
endif()

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Sub directory addition
add_subdirectory(LoggingModule)
add_subdirectory(SensorModule)
add_subdirectory(ZmqPublisher)
add_subdirectory(ViewerModule)

if(YOUTH_ENABLE_ALGORITHM)
    add_subdirectory(AlgorithmModule)
endif()

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
    ZmqPublisherLib
    OpenGL::GL
    ${GLU_LIB}
    GLEW::GLEW
    GLUT::GLUT
    stdc++
    pthread
    rt
)

if(YOUTH_ENABLE_ALGORITHM)
    target_link_libraries(Youth AlgorithmModuleLib)
endif()
