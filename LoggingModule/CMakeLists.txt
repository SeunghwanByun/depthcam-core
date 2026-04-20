# CMakeLists.txt of LoggingModule

add_library(LoggingModuleLib
    loggingModule.c
    loggingModule.h
)

target_link_libraries(LoggingModuleLib
    pthread
)