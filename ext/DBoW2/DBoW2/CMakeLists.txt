add_library(slam_ext_dbow2_dbow2 
    SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/BowVector.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/FeatureVector.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/FORB.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/ScoringObject.cpp
)

target_include_directories(slam_ext_dbow2_dbow2 
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/ext/DBoW2/DUtils/include
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(slam_ext_dbow2_dbow2 
    PUBLIC
        ${OpenCV_LIBS}
        Boost::serialization
)
