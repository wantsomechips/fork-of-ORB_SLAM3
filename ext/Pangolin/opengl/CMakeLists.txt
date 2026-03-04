include(EmbedBinaryFiles)
include(EmbedShaderFiles)

file(GLOB PANGOLIN_OPENGL_FONTS "${CMAKE_CURRENT_LIST_DIR}/src/fonts/*.ttf")
file(GLOB_RECURSE PANGOLIN_OPENGL_SHADERS "${CMAKE_CURRENT_LIST_DIR}/shaders/*.glsl*")

embed_binary_files_rule(${CMAKE_CURRENT_BINARY_DIR}/fonts.cpp "${PANGOLIN_OPENGL_FONTS}")
embed_shader_files_rule(${CMAKE_CURRENT_BINARY_DIR}/shaders.cpp "${PANGOLIN_OPENGL_SHADERS}")

add_library(
    slam_ext_pangolin_opengl
    SHARED
        src/glchar.cpp
        src/gldraw.cpp
        src/glfont.cpp
        src/gltext.cpp
        src/glpangoglu.cpp
        src/gltexturecache.cpp
        src/viewport.cpp
        src/opengl_render_state.cpp
        src/stb_truetype.h
        ${CMAKE_CURRENT_BINARY_DIR}/fonts.cpp
        ${CMAKE_CURRENT_BINARY_DIR}/shaders.cpp
)

find_package(Eigen3 REQUIRED NO_MODULE)
target_compile_definitions(slam_ext_pangolin_opengl PUBLIC HAVE_EIGEN)
target_link_libraries(
    slam_ext_pangolin_opengl
    PUBLIC
        slam_ext_pangolin_core
        slam_ext_pangolin_image
        Eigen3::Eigen
)

if(EMSCRIPTEN)
    target_compile_definitions(slam_ext_pangolin_opengl PUBLIC HAVE_GLES HAVE_GLES_2 HAVE_GLEW)
    target_sources(slam_ext_pangolin_opengl PRIVATE src/compat/gl2engine.cpp)
else()
    if(_LINUX_)
        set(OpenGL_GL_PREFERENCE "GLVND")
        find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL)
        find_package(epoxy REQUIRED)
        target_link_libraries(slam_ext_pangolin_opengl PUBLIC OpenGL::OpenGL OpenGL::EGL)
        target_link_libraries(slam_ext_pangolin_opengl PUBLIC ${epoxy_LIBRARIES})
        target_include_directories(slam_ext_pangolin_opengl PUBLIC ${epoxy_INCLUDE_DIRS})
        target_compile_definitions(slam_ext_pangolin_opengl PUBLIC HAVE_EPOXY)
    else()
        find_package(OpenGL REQUIRED)
        find_package(GLEW REQUIRED)
        target_include_directories(slam_ext_pangolin_opengl PUBLIC ${GLEW_INCLUDE_DIR})
        target_link_libraries(slam_ext_pangolin_opengl PUBLIC ${GLEW_LIBRARY})
        target_link_libraries(slam_ext_pangolin_opengl PUBLIC ${OPENGL_LIBRARIES})
        target_include_directories(slam_ext_pangolin_opengl PUBLIC ${OPENGL_INCLUDE_DIR})
        target_compile_definitions(slam_ext_pangolin_opengl PUBLIC HAVE_GLEW)
    endif()
endif()

target_compile_features(slam_ext_pangolin_opengl PUBLIC cxx_std_17)
target_include_directories(slam_ext_pangolin_opengl PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
set_target_properties(
    slam_ext_pangolin_opengl
    PROPERTIES VERSION ${PANGOLIN_VERSION} SOVERSION ${PANGOLIN_VERSION_MAJOR}
)
