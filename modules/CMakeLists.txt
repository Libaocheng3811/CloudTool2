set(LibSrcs
        features.cpp
        filters.cpp
        keypoints.cpp
        registration.cpp
        csffilter.cpp
        vegfilter.cpp
        )
set(LibHdrs
        features.h
        filters.h
        keypoints.h
        registration.h
        csffilter.h
        vegfilter.h
        utils.h)

add_library(ct_modules SHARED ${LibSrcs} ${LibHdrs})

target_compile_options(ct_modules PRIVATE /bigobj)

# 在 CMake 中，Target（目标）的名字是全局可见的逻辑名称，而不是物理文件路径，并不是一个文件名，而是一个全局对象名
target_link_libraries(ct_modules PRIVATE ct_core CSF_Lib)
target_link_libraries(ct_modules PRIVATE Qt${QT_VERSION_MAJOR}::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

# 在 CMake 构建过程中为 t_modules 库定义一个预处理宏。
target_compile_definitions(ct_modules PRIVATE CT_LIBRARY)

# 设置共享库的公共头文件属性，在引用这个库的时候就可以使用库中的函数
set_target_properties(ct_modules PROPERTIES PUBLIC_HEADER "${LibHdrs}")

install(TARGETS ct_modules PUBLIC_HEADER DESTINATION include/modules)
