file(GLOB_RECURS CONFIGURE_DEPENDS
        "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

        "${CMAKE_CURRENT_SOURCE_DIR}/*.h")

)

target_include_directories(${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(ct_algorithm
PRIVATE ct_core Qt5::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})
