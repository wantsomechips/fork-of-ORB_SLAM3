add_library(
    slam_ext_pangolin_windowing
    SHARED
        src/window.cpp
)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    target_link_libraries(slam_ext_pangolin_windowing PRIVATE "-framework Cocoa")
    target_sources(
        slam_ext_pangolin_windowing
        PRIVATE
            src/display_osx.mm
            src/PangolinNSApplication.mm
            src/PangolinNSGLView.mm
    )
    target_compile_definitions(slam_ext_pangolin_windowing PUBLIC "PANGO_DEFAULT_WIN_URI=\"cocoa\"")
    target_compile_options(
        slam_ext_pangolin_windowing
        PRIVATE
            $<$<OR:$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:
            -Wno-deprecated-declarations>
    )
    PangolinRegisterFactory(WindowInterface OsxWindow)
elseif(WIN32 OR WIN64)
    target_sources(slam_ext_pangolin_windowing PRIVATE src/display_win.cpp)
    target_compile_definitions(slam_ext_pangolin_windowing PUBLIC "PANGO_DEFAULT_WIN_URI=\"winapi\"")
    PangolinRegisterFactory(WindowInterface WinWindow)
elseif(EMSCRIPTEN)
    target_sources(slam_ext_pangolin_windowing PRIVATE src/display_emscripten.cpp)
    target_compile_definitions(
        slam_ext_pangolin_windowing
        PUBLIC
            "PANGO_DEFAULT_WIN_URI=\"emscripten\""
    )
    PangolinRegisterFactory(WindowInterface EmscriptenWindow)
else()
    find_package(X11 QUIET)
    find_package(OpenGL QUIET COMPONENTS EGL)
    if(X11_FOUND AND OpenGL_EGL_FOUND)
        target_sources(slam_ext_pangolin_windowing PRIVATE src/display_x11.cpp)
        target_link_libraries(slam_ext_pangolin_windowing PRIVATE ${X11_LIBRARIES})
        target_include_directories(slam_ext_pangolin_windowing PRIVATE ${X11_INCLUDE_DIR})
        target_link_libraries(slam_ext_pangolin_windowing PRIVATE OpenGL::EGL)
        PangolinRegisterFactory(WindowInterface X11Window)
    endif()

    find_package(PkgConfig)
    pkg_check_modules(WAYLAND_CLIENT wayland-client QUIET)
    pkg_check_modules(wayland-protocols QUIET wayland-protocols>=1.13)
    if(WAYLAND_CLIENT_FOUND AND wayland-protocols_FOUND)
        pkg_check_modules(WAYLAND_EGL wayland-egl REQUIRED)
        pkg_check_modules(WAYLAND_CURSOR wayland-cursor REQUIRED)
        pkg_check_modules(xkbcommon REQUIRED xkbcommon)

        target_sources(slam_ext_pangolin_windowing PRIVATE src/display_wayland.cpp)
        target_link_libraries(
            slam_ext_pangolin_windowing
            PRIVATE
                ${X11_LIBRARIES}
                ${WAYLAND_CLIENT_LIBRARIES}
                ${WAYLAND_EGL_LIBRARIES}
                ${WAYLAND_CURSOR_LIBRARIES}
                ${xkbcommon_LIBRARIES}
        )

        pkg_get_variable(WAYLAND_PROTOCOLS_DIR wayland-protocols pkgdatadir)
        pkg_get_variable(WAYLAND_SCANNER wayland-scanner wayland_scanner)

        set(XDG_PROT_DEF "${WAYLAND_PROTOCOLS_DIR}/stable/xdg-shell/xdg-shell.xml")
        add_custom_command(
            OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/xdg-shell-client-protocol.h
            COMMAND ${WAYLAND_SCANNER} client-header ${XDG_PROT_DEF} xdg-shell-client-protocol.h
        )
        add_custom_command(
            OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/xdg-shell-protocol.c
            COMMAND ${WAYLAND_SCANNER} private-code ${XDG_PROT_DEF} xdg-shell-protocol.c
            DEPENDS xdg-shell-client-protocol.h
        )
        target_include_directories(slam_ext_pangolin_windowing PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
        target_sources(slam_ext_pangolin_windowing PRIVATE xdg-shell-protocol.c)

        target_compile_definitions(
            slam_ext_pangolin_windowing
            PUBLIC
                "PANGO_DEFAULT_WIN_URI=\"wayland\""
        )
        PangolinRegisterFactory(WindowInterface WaylandWindow)
    else()
        target_compile_definitions(slam_ext_pangolin_windowing PUBLIC "PANGO_DEFAULT_WIN_URI=\"x11\"")
    endif()
endif()

if(_LINUX_)
    set(OpenGL_GL_PREFERENCE "GLVND")
endif()
find_package(OpenGL QUIET COMPONENTS EGL)
if(OpenGL_EGL_FOUND)
    target_sources(slam_ext_pangolin_windowing PRIVATE src/display_headless.cpp)
    target_link_libraries(slam_ext_pangolin_windowing PRIVATE ${OPENGL_egl_LIBRARY})
    PangolinRegisterFactory(WindowInterface HeadlessWindow)
endif()

create_factory_registry_file("${CMAKE_CURRENT_BINARY_DIR}/include/pangolin/factory" WindowInterface)
target_sources(
    slam_ext_pangolin_windowing
    PRIVATE
        "${CMAKE_CURRENT_BINARY_DIR}/include/pangolin/factory/RegisterFactoriesWindowInterface.h"
)

target_link_libraries(
    slam_ext_pangolin_windowing
    PUBLIC
        slam_ext_pangolin_core
        slam_ext_pangolin_opengl
)
target_compile_features(slam_ext_pangolin_windowing PUBLIC cxx_std_17)
target_include_directories(
    slam_ext_pangolin_windowing
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/include
        ${CMAKE_CURRENT_BINARY_DIR}/include
)
set_target_properties(
    slam_ext_pangolin_windowing
    PROPERTIES VERSION ${PANGOLIN_VERSION} SOVERSION ${PANGOLIN_VERSION_MAJOR}
)
