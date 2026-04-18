# CMakeLists.txt of SensorModule

set(ASTRA_SDK_PATH "" CACHE PATH "Path to the Astra SDK root (contains include/ and lib/)")

add_library(SensorModuleLib
    astra_wrapper.cpp
    astra_wrapper.h
    sensorModule.c
    sensorModule.h
)

target_link_libraries(SensorModuleLib
    stdc++
    pthread
    rt
)

if(NOT ASTRA_SDK_PATH AND EXISTS "/astra/x86_64")
  set(ASTRA_SDK_PATH "/astra/x86_64")
endif()

if(ASTRA_SDK_PATH AND EXISTS "${ASTRA_SDK_PATH}/include" AND EXISTS "${ASTRA_SDK_PATH}/lib")
  find_library(ASTRA_CAPI_LIB astra SHARED PATHS "${ASTRA_SDK_PATH}/lib" NO_DEFAULT_PATH)
  find_library(ASTRA_CORE_LIB astra_core SHARED PATHS "${ASTRA_SDK_PATH}/lib" NO_DEFAULT_PATH)

  if(ASTRA_CAPI_LIB AND ASTRA_CORE_LIB)
    message(STATUS "Using Astra SDK from ${ASTRA_SDK_PATH}")
    target_compile_definitions(SensorModuleLib PRIVATE USE_ASTRA_SDK)
    target_include_directories(SensorModuleLib PUBLIC "${ASTRA_SDK_PATH}/include")
    target_link_directories(SensorModuleLib PUBLIC "${ASTRA_SDK_PATH}/lib")
    target_link_libraries(SensorModuleLib "${ASTRA_CAPI_LIB}" "${ASTRA_CORE_LIB}")
    return()
  endif()
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENNI2 REQUIRED IMPORTED_TARGET libopenni2)

message(STATUS "Astra SDK not found. Falling back to OpenNI2 backend.")
target_compile_definitions(SensorModuleLib PRIVATE USE_OPENNI2)
target_link_libraries(SensorModuleLib PkgConfig::OPENNI2)

