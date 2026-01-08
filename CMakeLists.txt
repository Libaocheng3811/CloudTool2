cmake_minimum_required(VERSION 3.28)

project(CloudTool2)

if (POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif ()

# 启用 Qt 的自动处理功能
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "")
    set(CMAKE_BUILD_TYPE Release)
endif ()

set(CMAKE_DEBUG_POSTFIX "d")


if (MSVC)
    # 设置 默认C++ 编译器标志
    set(CMAKE_CXX_FLAGS_DEFAULT "/DWIN32 /D_WINDOWS /W3 /GR /EHsc")

    # 若现有的 C++ 编译器标志与默认标志相同，则追加一些额外的标志
    if ("${CMAKE_BUILD_TYPE}" STREQUAL "${CMAKE_CXX_FLAGS_DEFAULT}")
        string(APPEND CMAKE_CXX_FLAGS " /fp:precise /bigobj /wd4996 /source-charset:utf-8 ")
        # 将 MSVC_MP 变量设置为 CPU 核心数，用于启用多处理器编译
        set(MSVC_MP ${CPUCores})

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

option(WITH_OPENMP "Build with parallelization using OpenMP" TRUE)

if (WITH_OPENMP)
    find_package(OpenMP COMPONENTS C CXX)
endif ()

# 如果找到OpenMP，追加 OpenMP 编译器标志
if (OpenMP_FOUND)
    string(APPEND CMAKE_C_FLAGS " ${OpenMP_C_FLAGS}")
    string(APPEND CMAKE_CXX_FLAGS " ${OpemMP_CXX_FLAGS}")
    string(APPEND CMAKE_EXE_LINKER_FLAGS " ${OpenMP_EXE_LINKER_FLAGS}")

    if (${CMAKE_VERSION} VERSION_LESS 3.7)
        message(STATUS "Found OpenMP")
    else ()
        message(STATUS "Found OpenMP, spec date ${OpenMP_CXX_SPEC_DATE}")
    endif ()
else ()
    message(STATUS "Not found OpenMP")
endif ()

# QT 是你为找到的 Qt 包指定的别名
set(CMAKE_PREFIX_PATH
    "D:/LibCommunity/Qt5.15/5.15.2/msvc2019_64"
)

find_package(QT NAMES Qt5 COMPONENTS Core Widgets OpenGL Gui REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

if (QT_FOUND)
    message(STATUS "Found QT version: ${Qt5_VERSION}.")
    message(STATUS "Qt5_DIR = ${Qt5_DIR}")
    message(STATUS "Qt5Widgets_DIR = ${Qt5Widgets_DIR}")
    message(STATUS "Qt5Core_DIR = ${Qt5Core_DIR}")

else ()
    message(STATUS "Qt5 is not found.")
endif ()

# vtk
find_package(VTK REQUIRED)
if (VTK_FOUND)
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

find_package(OpenMP REQUIRED)

# 2. 配置编译选项：仅编译静态库，不编译工具
set(LASZIP_BUILD_STATIC ON CACHE BOOL "Build static laszip" FORCE)
set(LASLIB_BUILD_STATIC ON CACHE BOOL "Build static laslib" FORCE)
set(BUILD_TOOLS OFF CACHE BOOL "Build tools" FORCE)

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

include_directories(${PROJECT_SOURCE_DIR})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install)

# add_definitions 命令用于向编译器添加编译选项或定义宏
add_definitions(-DROOT_PATH="${PROJECT_SOURCE_DIR}")
add_definitions(-DDATA_PATH="${PROJECT_SOURCE_DIR}/data/")

add_subdirectory(3rdparty/LASTools)
add_subdirectory(3rdparty/CSF)
#add_subdirectory(ui)
add_subdirectory(base)
add_subdirectory(cloudtool)
add_subdirectory(modules)
