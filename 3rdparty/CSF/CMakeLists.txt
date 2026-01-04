cmake_minimum_required(VERSION 3.10)

project(CSF_Lib)

find_package(OpenMP REQUIRED)

# 收集源文件
file(GLOB CSF_SOURCES "src/*.cpp")
file(GLOB CSF_HEADERS "src/*.h")

add_library(CSF_Lib STATIC ${CSF_SOURCES} ${CSF_HEADERS})

# 指定包含目录(让外部能include "CSF.h")
target_include_directories(CSF_Lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

# 链接OpenMP
if (OpenMP_FOUND)
    target_link_libraries(CSF_Lib PRIVATE OpenMP::OpenMP_CXX)
endif()