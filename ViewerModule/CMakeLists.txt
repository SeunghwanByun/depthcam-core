# ViewerModule 라이브러리 생성
add_library(ViewerModuleLib
    viewerModule.c
    viewerModule.h
)

# GLFW 찾기
find_package(glfw3 REQUIRED)

# GLEW 찾기
find_package(GLEW REQUIRED)

# OpenGL 찾기
find_package(OpenGL REQUIRED)
find_library(GLU_LIB GLU REQUIRED)

target_include_directories(ViewerModuleLib PUBLIC
  ${GLFW_INCLUDE_DIRS}
)

# Find GLUT for HUD text rendering
find_package(GLUT REQUIRED)

# Find link libraries
target_link_libraries(ViewerModuleLib
    glfw
    GLEW::GLEW
    OpenGL::GL
    ${GLU_LIB}
    GLUT::GLUT
    AlgorithmModuleLib
    X11
    stdc++
    pthread
    rt
    ZmqPublisherLib
)

# 컴파일 옵션 설정
target_compile_options(ViewerModuleLib PRIVATE -Wall -Wextra)

# C 표준 설정
set_target_properties(ViewerModuleLib PROPERTIES C_STANDARD 11)
