add_library(slam_orbslam3_optimizationCore STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Optimizer.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/G2oTypes.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/OptimizableTypes.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Sim3Solver.cc
)

target_compile_features(slam_orbslam3_optimizationCore PUBLIC cxx_std_17)

target_include_directories(slam_orbslam3_optimizationCore
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(slam_orbslam3_optimizationCore
    PUBLIC
        slam_orbslam3_commonCore
        slam_orbslam3_mapCore
        slam_ext_g2o
        slam_ext_sophus
    PRIVATE
        slam_ext_dbow2
)
