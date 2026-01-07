set(Srcs
        main.cpp
        mainwindow.cpp
        edit/color.cpp
        edit/boundingbox.cpp
        tool/cutting.cpp
        tool/pickpoints.cpp
        tool/filters.cpp
        tool/keypoints.cpp
        tool/rangeimage.cpp
        tool/registration.cpp
        tool/descriptor.cpp
        plugins/csfplugin.cpp
        plugins/vegplugin.cpp
        )

set(Hdrs
        mainwindow.h
        edit/color.h
        edit/boundingbox.h
        tool/cutting.h
        tool/pickpoints.h
        tool/filters.h
        tool/keypoints.h
        tool/rangeimage.h
        tool/registration.h
        tool/descriptor.h
        plugins/csfplugin.h
        plugins/vegplugin.h
        )

set(QRCs resources/res.qrc)


set(IRCs resources/logo/logo.rc)

find_package(VTK COMPONENTS
    vtkGUISupportQt
    vtkGUISupportQtOpenGL
    vtkRenderingGL2PSOpenGL2
    vtkRenderingVolumeOpenGL2)

add_executable(cloudtool WIN32 ${Srcs} ${Hdrs} ${QRCs} ${IRCs})

if (CMAKE_BUILD_TYPE MATCHES "Debug")
        set(DEBUG_SUFFIX "d")
endif ()

target_link_libraries(cloudtool PRIVATE ct_base ct_modules)
target_link_libraries(cloudtool PRIVATE Qt${QT_VERSION_MAJOR}::Widgets ${VTK_LIBRARIES} ${PCL_LIBRARIES})
