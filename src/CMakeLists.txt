# add_library(Viewer STATIC
#     Viewer/Viewer.cpp
# )

# target_include_directories(Viewer PUBLIC
#     ${CMAKE_CURRENT_LIST_DIR}/../include
# )

# target_link_libraries(Viewer 
#     PUBLIC
#         OpenGL::GL
#         GLEW::GLEW
#         glfw
#         Threads::Threads
# )

# target_compile_definitions(Viewer
#     PRIVATE
#         $<$<CONFIG:Debug>:SR_DEBUG=1>
#         $<$<CONFIG:Release>:SR_RELEASE=1>
# )

# if(SR_BUILD_EXAMPLES)
#     add_executable(Youth
#     main.cpp
#     )
#     target_link_libraries(Youth 
#         PRIVATE
#             Viewer
#     )
#     set_target_properties(Youth PROPERTIES
#         RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
#     )
# endif()

# 실행 파일 생성
add_executable(Youth 
    main.cpp
    Viewer/Viewer.cpp
    GL/Quad.cpp
    Sensor/DepthSource.cpp
)

# OpenGL, GLEW, GLFW 찾기
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

# include 디렉토리 (상위 include/ 폴더)
target_include_directories(Youth PRIVATE 
    ${CMAKE_SOURCE_DIR}/include
)

# 링크
target_link_libraries(Youth PRIVATE 
    OpenGL::GL
    GLEW::GLEW
    glfw
    Threads::Threads
)

pkg_check_modules(OPENNI2 QUIET IMPORTED_TARGET libopenni2)

if(OPENNI2_FOUND)
    message(STATUS "Using OpenNI2 backend for Youth")
    target_link_libraries(Youth PRIVATE PkgConfig::OPENNI2)
    target_compile_definitions(Youth PRIVATE USE_OPENNI2)
else()
    message(WARNING "OpenNI2 not found. Youth will use the dummy depth backend.")
endif()
