add_library(slam_ext_g2o_core
    SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hyper_graph_action.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hyper_graph.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/marginal_covariance_cholesky.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/matrix_structure.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/batch_stats.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/parameter.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/cache.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/optimizable_graph.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/solver.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/optimization_algorithm_factory.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/estimate_propagator.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/factory.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/sparse_optimizer.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hyper_dijkstra.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/parameter_container.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/optimization_algorithm.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/optimization_algorithm_with_hessian.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/optimization_algorithm_levenberg.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/optimization_algorithm_gauss_newton.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/jacobian_workspace.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/robust_kernel.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/robust_kernel_factory.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/robust_kernel_impl.cpp
)

target_include_directories(slam_ext_g2o_core
    SYSTEM PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/g2o/core
)

target_link_libraries(slam_ext_g2o_core
    PUBLIC
        slam_ext_g2o_stuff
        slam_ext_eigen
)

if(TARGET slam_ext_eigen)
    target_link_libraries(slam_ext_g2o_core
        PUBLIC
            slam_ext_eigen
    )
elseif(EIGEN3_INCLUDE_DIR)
    target_include_directories(slam_ext_g2o_core
        SYSTEM PUBLIC
            ${EIGEN3_INCLUDE_DIR}
    )
endif()
