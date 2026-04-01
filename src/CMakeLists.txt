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
        tool/sampling.cpp
        tool/python_console.cpp
        tool/python_editor.cpp
        plugins/csfplugin.cpp
        plugins/vegplugin.cpp
        plugins/changedetectplugin.cpp
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
        tool/sampling.h
        tool/python_console.h
        tool/python_editor.h
        plugins/csfplugin.h
        plugins/vegplugin.h
        plugins/changedetectplugin.h
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

target_link_libraries(cloudtool PRIVATE ct_core ct_widget ct_modules ct_python)
target_link_libraries(cloudtool PRIVATE Qt${QT_VERSION_MAJOR}::Widgets ${VTK_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(cloudtool PRIVATE pybind11::embed Python3::Python)