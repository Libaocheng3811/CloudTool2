# Dialog library (common UI components)
file(GLOB Dialog_SOURCES CONFIGURE_DEPENDS "dialog/*.cpp")
file(GLOB Dialog_HEADERS CONFIGURE_DEPENDS "dialog/*.h")
file(GLOB Dialog_FORMS CONFIGURE_DEPENDS "dialog/*.ui")

add_library(ct_common_ui STATIC ${Dialog_SOURCES} ${Dialog_HEADERS} ${Dialog_FORMS})

target_include_directories(ct_common_ui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ct_common_ui PUBLIC
    ct_core
    Qt${QT_VERSION_MAJOR}::Widgets
)

# Widget library (base classes + cloudtree)
set(Widget_Srcs
    base/customtree.cpp
    base/cloudtree.cpp
)

set(Widget_Hdrs
    base/customtree.h
    base/cloudtree.h
    base/customdock.h
    base/customdialog.h
)

add_library(ct_widget STATIC ${Widget_Srcs} ${Widget_Hdrs})

target_include_directories(ct_widget PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ct_widget
    PUBLIC
        ct_common_ui
        ct_core
        ct_viz
        ct_io
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Widgets
)
