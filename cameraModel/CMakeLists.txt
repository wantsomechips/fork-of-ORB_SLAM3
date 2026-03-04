add_library(slam_orbslam3_cameraModel STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Pinhole.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/KannalaBrandt8.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/TwoViewReconstruction.cc
)

target_compile_features(slam_orbslam3_cameraModel PUBLIC cxx_std_17)

target_include_directories(slam_orbslam3_cameraModel
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(slam_orbslam3_cameraModel
    PUBLIC
        slam_orbslam3_commonCore
    PRIVATE
        slam_ext_dbow2
)
