add_library(slam_ext_g2o_types
    SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/types_sba.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/types_six_dof_expmap.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/types_seven_dof_expmap.cpp
)

target_include_directories(slam_ext_g2o_types
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/g2o/types
)

target_link_libraries(slam_ext_g2o_types
    PUBLIC
        slam_ext_g2o_core
        slam_ext_g2o_stuff
)

if(TARGET slam_ext_eigen)
    target_link_libraries(slam_ext_g2o_types
        PUBLIC
            slam_ext_eigen
    )
elseif(EIGEN3_INCLUDE_DIR)
    target_include_directories(slam_ext_g2o_types
        PUBLIC
            ${EIGEN3_INCLUDE_DIR}
    )
endif()
