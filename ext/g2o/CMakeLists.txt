add_subdirectory(stuff/)
add_subdirectory(core/)
add_subdirectory(types/)
add_subdirectory(solvers/)

add_library(slam_ext_g2o INTERFACE)

target_link_libraries(slam_ext_g2o
    INTERFACE
        slam_ext_g2o_core
        slam_ext_g2o_types
        slam_ext_g2o_stuff
        slam_ext_g2o_solvers
)
