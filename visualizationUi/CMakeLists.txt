add_library(slam_orbslam3_visualizationUi STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/FrameDrawer.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MapDrawer.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Viewer.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/VisualizationModel.cc
)

target_compile_features(slam_orbslam3_visualizationUi PUBLIC cxx_std_17)

target_include_directories(slam_orbslam3_visualizationUi
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(slam_orbslam3_visualizationUi
    PUBLIC
        slam_orbslam3_configCore
        slam_orbslam3_mapCore
        slam_orbslam3_trackingFrontend
        slam_ext_pangolin
)
