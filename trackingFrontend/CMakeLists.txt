add_library(slam_orbslam3_trackingFrontend STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Tracking.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/System.cc
)

target_compile_features(slam_orbslam3_trackingFrontend PUBLIC cxx_std_17)

target_include_directories(slam_orbslam3_trackingFrontend
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(slam_orbslam3_trackingFrontend
    PUBLIC
        slam_orbslam3_commonCore
        slam_orbslam3_configCore
        slam_orbslam3_cameraModel
        slam_orbslam3_featureCore
        slam_orbslam3_mapCore
        slam_orbslam3_optimizationCore
        slam_ext_pangolin
    PRIVATE
        Boost::serialization
        OpenSSL::Crypto
)
