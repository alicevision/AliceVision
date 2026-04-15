# =============================================================================
# deps/ffmpeg.cmake
# Dependencies: vpx (optional)
# Provides:     FFMPEG_TARGET, FFMPEG_CMAKE_FLAGS
#
# NOTE: FFmpeg uses a custom configure script — make is the only build tool.
# =============================================================================

if(AV_BUILD_FFMPEG)
    set(FFMPEG_TARGET ffmpeg)

    # Explicitly set the install name of the dynamic library to @rpath
    if(APPLE AND ALICEVISION_USE_RPATH)
        set(APPLE_FFMPEG_RPATH_FLAG --install-name-dir=@rpath)
    else()
        set(APPLE_FFMPEG_RPATH_FLAG)
    endif()

    if(APPLE)
        if(CMAKE_OSX_ARCHITECTURES MATCHES "x86_64")
            set(APPLE_FFMPEG_ARCH_FLAGS --arch=x86_64 --enable-cross-compile --sysroot=${APPLE_SYSROOT})
        elseif(CMAKE_OSX_ARCHITECTURES MATCHES "arm64")
            set(APPLE_FFMPEG_ARCH_FLAGS --arch=aarch64 --enable-cross-compile --sysroot=${APPLE_SYSROOT})
        endif()
        set(FFMPEG_CFLAGS -I<INSTALL_DIR>/include\ ${APPLE_ARCH_FLAGS})
        set(FFMPEG_LDFLAGS -L<INSTALL_DIR>/${CMAKE_INSTALL_LIBDIR}\ ${APPLE_ARCH_FLAGS})
    else()
        set(FFMPEG_CFLAGS -I<INSTALL_DIR>/include)
        set(FFMPEG_LDFLAGS -L<INSTALL_DIR>/${CMAKE_INSTALL_LIBDIR})
    endif()

    ExternalProject_Add(${FFMPEG_TARGET}
        URL              ${DEP_FFMPEG_URL}
        URL_HASH         ${DEP_FFMPEG_HASH}
        DOWNLOAD_DIR     ${BUILD_DIR}/download/ffmpeg
        DOWNLOAD_NO_PROGRESS TRUE
        PREFIX           ${BUILD_DIR}
        BUILD_IN_SOURCE  0
        BUILD_ALWAYS     0
        UPDATE_COMMAND   ""
        SOURCE_DIR       ${CMAKE_CURRENT_BINARY_DIR}/ffmpeg
        INSTALL_DIR      ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            <SOURCE_DIR>/configure
                --prefix=<INSTALL_DIR>
                --extra-cflags=${FFMPEG_CFLAGS}
                --extra-ldflags=${FFMPEG_LDFLAGS}
                --enable-shared
                --disable-static
                --disable-gpl
                --enable-nonfree
                --enable-libvpx
                ${APPLE_FFMPEG_ARCH_FLAGS}
                ${APPLE_FFMPEG_RPATH_FLAG}
        BUILD_COMMAND make -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${VPX_TARGET}
    )

    set(FFMPEG_CMAKE_FLAGS -DCMAKE_PREFIX_PATH=${CMAKE_INSTALL_PREFIX};${CMAKE_PREFIX_PATH})
    av_register_dep(${FFMPEG_TARGET})
endif()
