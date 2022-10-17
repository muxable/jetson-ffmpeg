cmake_minimum_required(VERSION 3.9)

option(NVMPI_VERSION "Version to build as" "0.0.1")

project(nvmpi VERSION ${NVMPI_VERSION} DESCRIPTION "Jetson hardware encoder/decoder API")

set(CPACK_PACKAGE_NAME "muxable-libnvmpi")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
set(CPACK_PACKAGE_ARCHITECTURE "arm64")
set(CPACK_PACKAGE_VERSION ${NVMPI_VERSION})
set(CPACK_PACKAGE_CONTACT "Kevin Wang <kevin@muxable.com>")
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)

include(CPack)

set(CMAKE_C_COMPILER $ENV{CROSS_COMPILE}gcc)
set(CMAKE_CXX_COMPILER $ENV{CROSS_COMPILE}g++)

set(CMAKE_C_FLAGS“$ {CMAKE_C_FLAGS} -fPIC”)
set(CMAKE_CXX_FLAGS“$ {CMAKE_CXX_FLAGS} -fPIC”)
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath-link=$ENV{TARGET_ROOTFS}/usr/lib/aarch64-linux-gnu/tegra")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath-link=$ENV{TARGET_ROOTFS}/usr/local/cuda/lib64")

find_library(LIB_NVBUF nvbuf_utils PATHS $ENV{TARGET_ROOTFS}/usr/lib/aarch64-linux-gnu/tegra)
find_library(LIB_V4L2 nvv4l2 PATHS $ENV{TARGET_ROOTFS}/usr/lib/aarch64-linux-gnu/tegra)
find_library(LIB_NVJPEG nvjpeg PATHS $ENV{TARGET_ROOTFS}/usr/lib/aarch64-linux-gnu/tegra)
find_package (Threads)

add_library(nvmpi SHARED
    nvmpi_dec.cpp
    nvmpi_enc.cpp
    $ENV{TARGET_ROOTFS}/usr/src/jetson_multimedia_api/samples/common/classes/NvBuffer.cpp
    $ENV{TARGET_ROOTFS}/usr/src/jetson_multimedia_api/samples/common/classes/NvElement.cpp
    $ENV{TARGET_ROOTFS}/usr/src/jetson_multimedia_api/samples/common/classes/NvElementProfiler.cpp
    $ENV{TARGET_ROOTFS}/usr/src/jetson_multimedia_api/samples/common/classes/NvLogging.cpp
    $ENV{TARGET_ROOTFS}/usr/src/jetson_multimedia_api/samples/common/classes/NvV4l2Element.cpp
    $ENV{TARGET_ROOTFS}/usr/src/jetson_multimedia_api/samples/common/classes/NvV4l2ElementPlane.cpp
    $ENV{TARGET_ROOTFS}/usr/src/jetson_multimedia_api/samples/common/classes/NvVideoDecoder.cpp
    $ENV{TARGET_ROOTFS}/usr/src/jetson_multimedia_api/samples/common/classes/NvVideoEncoder.cpp
)

add_library(nvmpi_static STATIC
    nvmpi_dec.cpp
    nvmpi_enc.cpp
    $ENV{TARGET_ROOTFS}/usr/src/jetson_multimedia_api/samples/common/classes/NvBuffer.cpp
    $ENV{TARGET_ROOTFS}/usr/src/jetson_multimedia_api/samples/common/classes/NvElement.cpp
    $ENV{TARGET_ROOTFS}/usr/src/jetson_multimedia_api/samples/common/classes/NvElementProfiler.cpp
    $ENV{TARGET_ROOTFS}/usr/src/jetson_multimedia_api/samples/common/classes/NvLogging.cpp
    $ENV{TARGET_ROOTFS}/usr/src/jetson_multimedia_api/samples/common/classes/NvV4l2Element.cpp
    $ENV{TARGET_ROOTFS}/usr/src/jetson_multimedia_api/samples/common/classes/NvV4l2ElementPlane.cpp
    $ENV{TARGET_ROOTFS}/usr/src/jetson_multimedia_api/samples/common/classes/NvVideoDecoder.cpp
    $ENV{TARGET_ROOTFS}/usr/src/jetson_multimedia_api/samples/common/classes/NvVideoEncoder.cpp
)

set_target_properties(nvmpi_static PROPERTIES OUTPUT_NAME nvmpi)
set_target_properties(nvmpi PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(nvmpi PROPERTIES SOVERSION 1)
set_target_properties(nvmpi nvmpi_static PROPERTIES PUBLIC_HEADER nvmpi.h)
set_target_properties(nvmpi PROPERTIES LINK_FLAGS "-Wl,--no-as-needed")
target_link_libraries(nvmpi PRIVATE ${CMAKE_THREAD_LIBS_INIT} ${LIB_NVBUF}  ${LIB_V4L2} ${LIB_NVJPEG})
target_link_libraries(nvmpi PRIVATE ${CMAKE_THREAD_LIBS_INIT} ${LIB_NVBUF}  ${LIB_V4L2})
target_include_directories(nvmpi PRIVATE $ENV{TARGET_ROOTFS}/usr/src/jetson_multimedia_api/include)
target_include_directories(nvmpi PRIVATE $ENV{TARGET_ROOTFS}/usr/local/cuda/include)
target_include_directories(nvmpi_static PRIVATE $ENV{TARGET_ROOTFS}/usr/src/jetson_multimedia_api/include)
target_include_directories(nvmpi_static PRIVATE $ENV{TARGET_ROOTFS}/usr/local/cuda/include)
configure_file(nvmpi.pc.in nvmpi.pc @ONLY)
include(GNUInstallDirs)
install(TARGETS nvmpi nvmpi_static
	    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES ${CMAKE_BINARY_DIR}/nvmpi.pc
	    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)
install(FILES ${CMAKE_BINARY_DIR}/nvmpi.pc
	    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
