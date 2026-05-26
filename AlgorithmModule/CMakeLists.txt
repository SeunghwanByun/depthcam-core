cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED
    COMPONENTS core imgproc features2d calib3d)

add_library(AlgorithmModuleLib
    voModule.cpp
    voModule.h
)

target_include_directories(AlgorithmModuleLib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(AlgorithmModuleLib
    ${OpenCV_LIBS}
)

target_compile_options(AlgorithmModuleLib PRIVATE -Wall -Wextra -O2)

# Copy camera config to build directory for reference
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config/astra_orb_slam3_rgbd.yaml
    DESTINATION ${CMAKE_BINARY_DIR}/config)

file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/output)
