add_subdirectory(DBoW2/)
add_subdirectory(DUtils/)

add_library(slam_ext_dbow2 INTERFACE)

target_link_libraries(slam_ext_dbow2 
    INTERFACE
        slam_ext_dbow2_dbow2 
        slam_ext_dbow2_dutils
)
