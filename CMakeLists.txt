cmake_minimum_required(VERSION 3.16)
project(Youth LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# include 경로 전역 추가
include_directories((${CMAKE_CURRENT_SOURCE_DIR}/include))

# 하위 디렉토리 추가
add_subdirectory(src)