cmake_minimum_required(VERSION 3.28)
project(CloudTool2)

# 确保项目的 CMakeLists.txt 文件在不同版本的 CMake 中都能以相同的方式处理 find_package 命令的私有依赖项
if (POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif ()

# 启用 Qt 的自动处理功能
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)

# 设置了 C++ 语言的标准版本
# 指定必须使用 C++17 标准或更高版本。如果编译器不支持 C++17，CMake 将报错而不是降级到旧的标准。
# 禁用了编译器的扩展功能
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 是否定义了 CMAKE_BUILD_TYPE 变量,若未定义，则设置为release
if ("${CMAKE_BUILD_TYPE}" STREQUAL "")
    set(CMAKE_BUILD_TYPE Release)
endif ()

# 设置调试构建的后缀为 "d"
set(CMAKE_DEBUG_POSTFIX "d")

# 根据不同编译器环境设置不同的编译器标志
# 对于msvc编译器
if (MSVC)
    # 设置 默认C++ 编译器标志
    set(CMAKE_CXX_FLAGS_DEFAULT "/DWIN32 /D_WINDOWS /W3 /GR /EHsc")

    # 若现有的 C++ 编译器标志与默认标志相同，则追加一些额外的标志
    if ("${CMAKE_BUILD_TYPE}" STREQUAL "${CMAKE_CXX_FLAGS_DEFAULT}")
        string(APPEND CMAKE_CXX_FLAGS " /fp:precise /bigobj /wd4996 /source-charset:utf-8 ")
        # 将 MSVC_MP 变量设置为 CPU 核心数，用于启用多处理器编译
        set(MSVC_MP ${CPUCores})

        # 为旧版和新版本 CMake 设置多处理器编译标志
        if (CMAKE_VERSION VERSION_LESS 3.11.0)
            if (MSVC_MP EQUAL 0)
                string(APPEND CMAKE_CXX_FLAGS " /MP")
            elseif (MSVC_MP GREATER 1)
                string(APPEND CMAKE_CXX_FLAGS " /MP${MSVC_MP}")
            endif ()
        else ()
            if (MSVC_MP EQUAL 0)
                add_compile_options($<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:/MP>)
            elseif (MSVC_MP GREATER 1)
                add_compile_options($<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:/MP${MSVC_MP}>)
            endif ()
        endif ()
    endif ()
    # 对于GNU编译器
elseif (CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS_DEFAULT "")
    if (CMAKE_COMPILER_VERSION VERSION_GREATER 7)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wabi=11")
    else ()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wabi")
    endif ()
endif ()

#OpenMP (optional)
# 在项目中启用 OpenMP 支持，它允许程序利用多核处理器进行并行计算
option(WITH_OPENMP "Build with parallelization using OpenMP" TRUE)

if (WITH_OPENMP)
    find_package(OpenMP COMPONENTS C CXX)
endif ()

# 如果找到OpenMP，追加 OpenMP 编译器标志
if (OpenMP_FOUND)
    string(APPEND CMAKE_C_FLAGS " ${OpenMP_C_FLAGS}")
    string(APPEND CMAKE_CXX_FLAGS " ${OpemMP_CXX_FLAGS}")
    string(APPEND CMAKE_EXE_LINKER_FLAGS " ${OpenMP_EXE_LINKER_FLAGS}")

    # 根据cmake版本打印信息
    # CMake 版本小于 3.7，只打印 "Found OpenMP"
    # CMake 版本大于等于 3.7，打印 "Found OpenMP, spec date${OpenMP_CXX_SPEC_DATE}"
    if (${CMAKE_VERSION} VERSION_LESS 3.7)
        message(STATUS "Found OpenMP")
    else ()
        message(STATUS "Found OpenMP, spec date ${OpenMP_CXX_SPEC_DATE}")
    endif ()
else ()
    message(STATUS "Not found OpenMP")
endif ()

# qt
# QT 是你为找到的 Qt 包指定的别名
find_package(QT NAMES Qt5 COMPONENTS Core Widgets OpenGL Gui REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

if (QT_FOUND)
    # 打印信息的${Qt5_VERSION}说明QT只是你指定的别名，实际文件查找中还是Qt5
    message(STATUS "Found QT version: ${Qt5_VERSION}.")
else ()
    message(STATUS "Qt5 is not found.")
endif ()

# vtk
find_package(VTK REQUIRED)
if (VTK_FOUND)
    # VTK_USE_FILE 是一个由 VTK 的 CMake 配置文件设置的变量
    # 包含VTK_USE_FILE这个文件时，会设置一系列变量，
    # 包括VTK_INCLUDE_DIRS(VTK头文件的路径)、VTK_LIBRARIES(需要链接的 VTK 库)、VTK_DEFINITIONS(编译 VTK 时可能需要的定义)
    # include命令用于包含一个 CMake 脚本文件,不仅可以用来添加头文件搜索路径，还可以用来执行其他 CMake 命令，比如设置变量、定义宏、添加编译器标志
    include(${VTK_USE_FILE})
    message(STATUS "Found VTK version: ${VTK_VERSION}.")
else ()
    message(STATUS "VTK is not found.")
endif ()

# pcl
find_package(PCL REQUIRED)

if (PCL_FOUND)
    # include_directories命令专门用于添加头文件搜索路径
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    message(STATUS "Found PCL version: ${PCL_VERSION}.")
else ()
    message(STATUS "PCL is not found.")
endif ()

# camera sdk
# 1. Azure Kinect
set(AzureKinect_DIR "${PROJECT_SOURCE_DIR}/camera/AzureKinect")
include_directories("${AzureKinect_DIR}/include")
if(WIN32)
    link_directories("${AzureKinect_DIR}/lib/win32")
elseif(UNIX)
    link_directories("${AzureKinect_DIR}/lib/linux")
endif()

# 2. Photoneo SDK
set(Photoneo_DIR "${PROJECT_SOURCE_DIR}/camera/Photoneo")
include_directories("${Photoneo_DIR}/include")
if(WIN32)
    link_directories("${Photoneo_DIR}/lib/win32")
elseif(UNIX)
    link_directories("${Photoneo_DIR}/lib/linux")
endif()

message(STATUS "Build Type: " ${CMAKE_BUILD_TYPE})

# PROJECT_SOURCE_DIR 是一个内置变量，它指定了项目的源代码目录
# 使用 `project()` 命令定义项目时，CMake 会自动设置 `PROJECT_SOURCE_DIR` 和与之对应的 `PROJECT_BINARY_DIR` 变量。
# `PROJECT_BINARY_DIR` 指向构建目录
include_directories(${PROJECT_SOURCE_DIR})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install)

# add_definitions 命令用于向编译器添加编译选项或定义宏
add_definitions(-DROOT_PATH="${PROJECT_SOURCE_DIR}")
add_definitions(-DDATA_PATH="${PROJECT_SOURCE_DIR}/data/")

add_subdirectory(base)
add_subdirectory(cloudtool)
add_subdirectory(modules)
