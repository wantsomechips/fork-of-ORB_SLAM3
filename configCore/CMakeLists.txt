add_library(slam_orbslam3_configCore STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Settings.cc
)

target_compile_features(slam_orbslam3_configCore PUBLIC cxx_std_17)

target_include_directories(slam_orbslam3_configCore
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(slam_orbslam3_configCore
    PUBLIC
        slam_orbslam3_commonCore
        slam_orbslam3_cameraModel
        ${OpenCV_LIBS}
)
