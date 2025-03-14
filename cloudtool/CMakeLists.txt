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
        )

# 定义资源变量， .qrc是Qt格式的资源文件，.rc文件是Window的资源文件格式
# .rc文件通过资源编译器（rc.exe）被编译成.res文件，然后在链接阶段被整合到可执行文件（.exe）中。这些资源可以通过资源标识符在代码中访问和使用
set(QRCs resources/res.qrc)

# logo.rc中 IDI_ICON1 ICON DISCARDABLE "logo.ico" ,IDI_ICON1是资源名称，ICON资源类型，表示图标资源，DISCARDABLE关键字表示这个资源是可丢弃的
# 当你使用.rc文件时，你不需要在代码中设置窗口图标，因为.rc文件中定义的应用程序图标资源会自动被Windows使用作为窗口的图标
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
