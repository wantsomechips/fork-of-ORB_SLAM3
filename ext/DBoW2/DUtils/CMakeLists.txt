add_library(slam_ext_dbow2_dutils 
    SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Random.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Timestamp.cpp
)

target_include_directories(slam_ext_dbow2_dutils
    SYSTEM PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)