add_library(slam_orbslam3_mapCore STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Map.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MapPoint.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/KeyFrame.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/KeyFrameDatabase.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Atlas.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Frame.cc
)

target_compile_features(slam_orbslam3_mapCore PUBLIC cxx_std_17)

target_include_directories(slam_orbslam3_mapCore
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(slam_orbslam3_mapCore
    PUBLIC
        slam_orbslam3_commonCore
        slam_orbslam3_configCore
        slam_orbslam3_cameraModel
        slam_orbslam3_featureCore
        slam_ext_dbow2
    PRIVATE
        slam_ext_sophus
        Boost::serialization
        OpenSSL::Crypto
)
