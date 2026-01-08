set(LibSrcs
        cloud.cpp
        cloudview.cpp
        common.cpp
        console.cpp
        fileio.cpp
        )

set(LibHdrs
        exports.h
        cloud.h
        cloudview.h
        common.h
        console.h
        fileio.h
        )

find_package(VTK COMPONENTS
    vtkGUISupportQt
    vtkGUISupportQtOpenGL
    vtkRenderingGL2PSOpenGL2
    vtkRenderingVolumeOpenGL2)

# LASlib头文件路径
set(LAS_INCLUDE_DIRS
    "${CMAKE_SOURCE_DIR}/3rdparty/LAStools/LASlib/inc"
    "${CMAKE_SOURCE_DIR}/3rdparty/LAStools/LASzip/src"
)

add_library(ct_core SHARED ${LibSrcs} ${LibHdrs})

target_include_directories(ct_core PUBLIC ${LAS_INCLUDE_DIRS})

target_link_libraries(ct_core
        PUBLIC
        Qt${QT_VERSION_MAJOR}::Widgets
        ${PCL_LIBRARIES}
        ${VTK_LIBRARIES}
        PRIVATE
        LASlib
        OpenMP::OpenMP_CXX)

target_compile_definitions(ct_core PRIVATE CT_LIBRARY)

set_target_properties(ct_core PROPERTIES PUBLIC_HEADER "${LibHdrs}")

install(TARGETS ct_core PUBLIC_HEADER DESTINATION inlcude/core)