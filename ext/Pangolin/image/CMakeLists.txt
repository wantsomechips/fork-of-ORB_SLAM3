add_library(
    slam_ext_pangolin_image
    SHARED
        src/pixel_format.cpp
        src/image_io.cpp
        src/image_io_exr.cpp
        src/image_io_jpg.cpp
        src/image_io_lz4.cpp
        src/image_io_packed12bit.cpp
        src/image_io_pango.cpp
        src/image_io_png.cpp
        src/image_io_ppm.cpp
        src/image_io_raw.cpp
        src/image_io_tga.cpp
        src/image_io_bmp.cpp
        src/image_io_zstd.cpp
        src/image_io_libraw.cpp
        src/image_io_tiff.cpp
)

find_package(Eigen3 REQUIRED NO_MODULE)
target_compile_definitions(slam_ext_pangolin_image PUBLIC HAVE_EIGEN)
target_link_libraries(slam_ext_pangolin_image PUBLIC slam_ext_pangolin_core Eigen3::Eigen)

find_package(PNG QUIET)
if(PNG_FOUND)
    target_compile_definitions(slam_ext_pangolin_image PRIVATE HAVE_PNG)
    target_include_directories(slam_ext_pangolin_image PRIVATE ${PNG_INCLUDE_DIR})
    target_link_libraries(slam_ext_pangolin_image PRIVATE ${PNG_LIBRARY} ${ZLIB_LIBRARY})
endif()

find_package(JPEG QUIET)
if(JPEG_FOUND)
    target_compile_definitions(slam_ext_pangolin_image PRIVATE HAVE_JPEG)
    target_include_directories(slam_ext_pangolin_image PRIVATE ${JPEG_INCLUDE_DIR})
    target_link_libraries(slam_ext_pangolin_image PRIVATE ${JPEG_LIBRARY})
endif()

find_package(TIFF QUIET)
if(TIFF_FOUND)
    target_compile_definitions(slam_ext_pangolin_image PRIVATE HAVE_LIBTIFF)
    target_link_libraries(slam_ext_pangolin_image PRIVATE TIFF::TIFF)
endif()

find_package(OpenEXR QUIET)
if(OpenEXR_FOUND)
    target_compile_definitions(slam_ext_pangolin_image PRIVATE HAVE_OPENEXR)
    target_include_directories(slam_ext_pangolin_image PRIVATE ${OpenEXR_INCLUDE_DIR})
    target_link_libraries(slam_ext_pangolin_image PRIVATE ${OpenEXR_LIBRARY})
    target_compile_options(
        slam_ext_pangolin_image
        PRIVATE
            $<$<OR:$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:
            -Wno-deprecated-register -Wno-deprecated>
    )
endif()

find_package(Lz4 QUIET)
if(Lz4_FOUND)
    target_compile_definitions(slam_ext_pangolin_image PRIVATE HAVE_LZ4)
    target_include_directories(slam_ext_pangolin_image PRIVATE ${Lz4_INCLUDE_DIRS})
    target_link_libraries(slam_ext_pangolin_image PRIVATE ${Lz4_LIBRARIES})
endif()

find_package(zstd QUIET)
if(zstd_FOUND)
    target_compile_definitions(slam_ext_pangolin_image PRIVATE HAVE_ZSTD)
    target_include_directories(slam_ext_pangolin_image PRIVATE ${zstd_INCLUDE_DIR})
    target_link_libraries(slam_ext_pangolin_image PRIVATE ${zstd_LIBRARY})
endif()

find_package(libraw QUIET)
if(libraw_FOUND)
    target_compile_definitions(slam_ext_pangolin_image PRIVATE HAVE_LIBRAW)
    target_include_directories(slam_ext_pangolin_image PRIVATE ${libraw_INCLUDE_DIR})
    target_link_libraries(slam_ext_pangolin_image PRIVATE ${libraw_LIBRARIES})
endif()

target_compile_features(slam_ext_pangolin_image PUBLIC cxx_std_17)
target_include_directories(slam_ext_pangolin_image SYSTEM PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
set_target_properties(
    slam_ext_pangolin_image
    PROPERTIES VERSION ${PANGOLIN_VERSION} SOVERSION ${PANGOLIN_VERSION_MAJOR}
)
