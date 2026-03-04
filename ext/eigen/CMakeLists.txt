add_library(slam_ext_eigen INTERFACE)

target_include_directories(slam_ext_eigen
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/
)