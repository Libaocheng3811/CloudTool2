set(LibSrcs
        cloud.cpp
        cloudtree.cpp
        cloudview.cpp
        common.cpp
        console.cpp
        customtree.cpp
        fileio.cpp
        )

set(LibHdrs
        exports.h
        cloud.h
        cloudtree.h
        cloudview.h
        common.h
        console.h
        customtree.h
        fileio.h
        customdialog.h
        customdock.h
        fieldmappingdialog.h
        txtimportdialog.h
        txtexportdialog.h
        processingdialog.h
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

add_library(ct_base SHARED ${LibSrcs} ${LibHdrs})

target_include_directories(ct_base PUBLIC ${LAS_INCLUDE_DIRS})

target_link_libraries(ct_base PRIVATE
        Qt${QT_VERSION_MAJOR}::Widgets
        ${PCL_LIBRARIES}
        ${VTK_LIBRARIES}
        LASlib
        OpenMP::OpenMP_CXX)

target_compile_definitions(ct_base PRIVATE CT_LIBRARY)

set_target_properties(ct_base PROPERTIES PUBLIC_HEADER "${LibHdrs}")

install(TARGETS ct_base PUBLIC_HEADER DESTINATION inlcude/base)