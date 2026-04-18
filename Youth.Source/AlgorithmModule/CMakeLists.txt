# CMakeLists.txt of Algorithm

cmake_minimum_required(VERSION 3.10)

set(ORB_SLAM3_ROOT_DIR "/usr/local/ORB_SLAM3" CACHE PATH "Path to the ORB_SLAM3 installation")

# 컴파일러 플래그 설정
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ORB_SLAM3_INCLUDE_DIR "${ORB_SLAM3_ROOT_DIR}/include")
set(ORB_SLAM3_LIB_DIR "${ORB_SLAM3_ROOT_DIR}/lib")

# OpenCV 패키지 찾기
find_package(OpenCV REQUIRED)

# PCL 패키지 찾기 (포인트 클라우드 처리용)
find_package(PCL QUIET)

# 헤더 파일 경로 추가
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${ORB_SLAM3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

# 링크 디렉토리 추가
link_directories(
  ${ORB_SLAM3_LIB_DIR}
  ${PCL_LIBRARY_DIRS}
)

#
add_library(AlgorithmModuleLib
  SLAM.cpp
  SLAM.h
  algorithmModule.c
  algorithmModule.h
)

target_link_libraries(AlgorithmModuleLib
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
  ORB_SLAM3)

# 설정 파일 복사 (빌드 디렉토리에)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config/astra_orb_slam3_rgbd.yaml
  DESTINATION ${CMAKE_BINARY_DIR}/config)

# ORB 어휘 파일 복사
if(EXISTS "${ORB_SLAM3_ROOT_DIR}/Vocabulary/ORBvoc.txt")
  file(COPY ${ORB_SLAM3_ROOT_DIR}/Vocabulary/ORBvoc.txt
    DESTINATION ${CMAKE_BINARY_DIR}/config)
else()
  message(WARNING "ORB vocabulary file was not found at ${ORB_SLAM3_ROOT_DIR}/Vocabulary/ORBvoc.txt")
endif()

# 출력 디렉토리 생성
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/output)
