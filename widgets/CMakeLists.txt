file(GLOB UI_SOURCES "common_ui/*.cpp")
file(GLOB UI_HEADERS "common_ui/*.h")
file(GLOB UI_FORMS "common_ui/*.ui")

add_library(ct_common_ui STATIC ${UI_SOURCES} ${UI_HEADERS} ${UI_FORMS})

target_link_libraries(ct_common_ui PUBLIC
        ct_core
        Qt${QT_VERSION_MAJOR}::Widgets)

# 包含路径设置,  #include "common_ui/customdialog.h"
target_include_directories(ct_common_ui PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/common_ui)

#target_compile_definitions(ct_common_ui PRIVATE CT_LIBRARY)


set(Widget_Srcs
        customtree.cpp
        cloudtree.cpp)

set(Widget_Hdrs
        customtree.h
        cloudtree.h
        customdock.h
        customdialog.h)

add_library(ct_widget STATIC ${Widget_Srcs} ${Widget_Hdrs})

target_link_libraries(ct_widget
        PUBLIC
            ct_common_ui
            ct_core
        PRIVATE
            Qt${QT_VERSION_MAJOR}::Widgets
)

target_include_directories(ct_widget PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

#target_compile_definitions(ct_widget PRIVATE CT_LIBRARY)

set_target_properties(ct_widget PROPERTIES PUBLIC_HEADER "${Widget_Hdrs}")

install(TARGETS ct_widget
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include/widgets
)

install(TARGETS ct_common_ui
        ARCHIVE DESTINATION lib
)