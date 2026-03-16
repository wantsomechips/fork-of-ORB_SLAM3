add_library(slam_ext_g2o_stuff
    SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/src/timeutil.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/os_specific.c
        ${CMAKE_CURRENT_SOURCE_DIR}/src/string_tools.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/property.cpp
)

target_include_directories(slam_ext_g2o_stuff
    SYSTEM PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/g2o/stuff
)

if(TARGET slam_ext_eigen)
    target_link_libraries(slam_ext_g2o_stuff
        PUBLIC
            slam_ext_eigen
    )
elseif(EIGEN3_INCLUDE_DIR)
    target_include_directories(slam_ext_g2o_stuff
        SYSTEM PUBLIC
            ${EIGEN3_INCLUDE_DIR}
    )
endif()
