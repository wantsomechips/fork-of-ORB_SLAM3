add_library(slam_ext_g2o_solvers INTERFACE)

target_include_directories(slam_ext_g2o_solvers
    SYSTEM INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(slam_ext_g2o_solvers
    INTERFACE
        slam_ext_g2o_core
        slam_ext_g2o_stuff
)

if(TARGET slam_ext_eigen)
    target_link_libraries(slam_ext_g2o_solvers
        INTERFACE
            slam_ext_eigen
    )
elseif(EIGEN3_INCLUDE_DIR)
    target_include_directories(slam_ext_g2o_solvers
        SYSTEM INTERFACE
            ${EIGEN3_INCLUDE_DIR}
    )
endif()
