add_library(slam_orbslam3_commonCore STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Config.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Converter.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/GeometricTools.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ImuTypes.cc
)

target_compile_features(slam_orbslam3_commonCore PUBLIC cxx_std_17)

target_include_directories(slam_orbslam3_commonCore
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(slam_orbslam3_commonCore
    PUBLIC
        slam_ext_eigen
        slam_ext_sophus
        slam_ext_g2o
        Boost::serialization
        ${OpenCV_LIBS}
)
