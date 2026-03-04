include(EmbedBinaryFiles)

set(PANGOLIN_DEFAULT_FONT ${CMAKE_CURRENT_LIST_DIR}/../opengl/src/fonts/AnonymousPro.ttf)
embed_binary_files_rule(${CMAKE_CURRENT_BINARY_DIR}/fonts.cpp "${PANGOLIN_DEFAULT_FONT}")

add_library(
    slam_ext_pangolin_display
    SHARED
        src/display.cpp
        src/process.cpp
        src/pangolin_gl.cpp
        src/handler.cpp
        src/handler_image.cpp
        src/handler_glbuffer.cpp
        src/view.cpp
        src/widgets.cpp
        src/image_view.cpp
        src/ConsoleView.cpp
        src/default_font.cpp
        ${CMAKE_CURRENT_BINARY_DIR}/fonts.cpp
)

target_link_libraries(
    slam_ext_pangolin_display
    PUBLIC
        slam_ext_pangolin_core
        slam_ext_pangolin_opengl
        slam_ext_pangolin_windowing
        slam_ext_pangolin_vars
)
target_compile_features(slam_ext_pangolin_display PUBLIC cxx_std_17)
target_include_directories(slam_ext_pangolin_display PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
set_target_properties(
    slam_ext_pangolin_display
    PROPERTIES VERSION ${PANGOLIN_VERSION} SOVERSION ${PANGOLIN_VERSION_MAJOR}
)
