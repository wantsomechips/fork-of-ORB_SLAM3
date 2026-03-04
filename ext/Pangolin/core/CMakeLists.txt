add_library(
    slam_ext_pangolin_core
    SHARED
        src/file_extension.cpp
        src/file_utils.cpp
        src/sigstate.cpp
        src/threadedfilebuf.cpp
        src/avx_math.cpp
        src/uri.cpp
        src/param_set.cpp
        src/factory/factory_registry.cpp
        src/factory/factory_help.cpp
)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_compile_definitions(slam_ext_pangolin_core PUBLIC "_OSX_")
elseif(WIN32 OR WIN64)
    target_compile_definitions(slam_ext_pangolin_core PUBLIC "_WIN_")
elseif(EMSCRIPTEN)
    target_compile_definitions(slam_ext_pangolin_core PUBLIC "_EMSCRIPTEN_")
else()
    target_compile_definitions(slam_ext_pangolin_core PUBLIC "_LINUX_")
endif()

if(UNIX)
    target_sources(
        slam_ext_pangolin_core
        PRIVATE
            src/posix/condition_variable.cpp
            src/posix/semaphore.cpp
            src/posix/shared_memory_buffer.cpp
    )
    if(NOT APPLE)
        target_link_libraries(slam_ext_pangolin_core PUBLIC rt)
    endif()
elseif(WIN32 OR WIN64)
    target_link_libraries(slam_ext_pangolin_core PRIVATE shlwapi.lib)
endif()

find_package(Threads QUIET)
if(Threads_FOUND)
    target_link_libraries(slam_ext_pangolin_core PUBLIC Threads::Threads)
endif()

target_compile_features(slam_ext_pangolin_core PUBLIC cxx_std_17)
target_include_directories(slam_ext_pangolin_core PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
set_target_properties(
    slam_ext_pangolin_core
    PROPERTIES VERSION ${PANGOLIN_VERSION} SOVERSION ${PANGOLIN_VERSION_MAJOR}
)
