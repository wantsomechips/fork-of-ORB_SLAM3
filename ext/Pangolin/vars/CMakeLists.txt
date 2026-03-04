add_library(
    slam_ext_pangolin_vars
    SHARED
        src/vars.cpp
        src/varstate.cpp
)

target_link_libraries(slam_ext_pangolin_vars PUBLIC slam_ext_pangolin_core)
target_compile_features(slam_ext_pangolin_vars PUBLIC cxx_std_17)
target_include_directories(slam_ext_pangolin_vars PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
set_target_properties(
    slam_ext_pangolin_vars
    PROPERTIES VERSION ${PANGOLIN_VERSION} SOVERSION ${PANGOLIN_VERSION_MAJOR}
)
