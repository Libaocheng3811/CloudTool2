file(GLOB Python_Srcs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
file(GLOB Python_Hdrs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

add_library(ct_python OBJECT ${Python_Srcs} ${Python_Hdrs})

target_link_libraries(ct_python
    PUBLIC
    Qt${QT_VERSION_MAJOR}::Widgets
    pybind11::embed
    Python3::Python
    PRIVATE
    ct_core
)

target_include_directories(ct_python PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..)
