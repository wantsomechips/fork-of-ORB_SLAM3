set(PANGOLIN_CMAKE_DIR "${CMAKE_CURRENT_LIST_DIR}/cmake")
set(PANGOLIN_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}")
list(APPEND CMAKE_MODULE_PATH "${PANGOLIN_CMAKE_DIR}")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

include(SetPlatformVars)
include(PangolinFactory)

add_subdirectory(core/)
add_subdirectory(image/)
add_subdirectory(opengl/)
add_subdirectory(windowing/)
add_subdirectory(vars/)
add_subdirectory(display/)

add_library(slam_ext_pangolin INTERFACE)
target_link_libraries(
    slam_ext_pangolin
    INTERFACE
        slam_ext_pangolin_core
        slam_ext_pangolin_image
        slam_ext_pangolin_opengl
        slam_ext_pangolin_windowing
        slam_ext_pangolin_vars
        slam_ext_pangolin_display
)
