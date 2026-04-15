# =============================================================================
# deps/vpx.cmake
# Dependencies: (none)
# Provides:     VPX_TARGET
#
# NOTE: libvpx uses a custom configure script — make is the only build tool.
# Only included when AV_BUILD_FFMPEG is ON (vpx is a codec for ffmpeg).
# =============================================================================

if(AV_BUILD_VPX AND AV_BUILD_FFMPEG)
    set(VPX_TARGET libvpx)

    if(APPLE AND ALICEVISION_USE_RPATH)
      set(VPX_APPLE_LDFLAGS_ENV_CMD ${CMAKE_COMMAND} -E env LDFLAGS=-Wl,-install_name,@rpath/libvpx.dylib)
    else()
      set(VPX_APPLE_LDFLAGS_ENV_CMD)
    endif()

    # This is currently required until libVPX properly supports macOS 26 Tahoe.
    # The configure.sh script only detects Darwin versions up to 24.X.X,
    # but macOS 26 Tahoe is version 25.X.X, causing the configure script
    # to fall back to generic-gnu, which assumes Linux and therefore pulls
    # in the wrong linker flags.
    # If we detect macOS 26 or higher, we explicitly set the toolchain to
    # be (x86_64/arm64)-darwin24-gcc. For this, use CMAKE_SYSTEM_VERSION.
    set(VPX_TOOLCHAIN_FLAG)
    if(APPLE)
        if(CMAKE_SYSTEM_VERSION VERSION_GREATER_EQUAL 25) # Tahoe and later
            if(CMAKE_OSX_ARCHITECTURES MATCHES "x86_64")
                set(VPX_TOOLCHAIN_FLAG --target=x86_64-darwin24-gcc)
            elseif(CMAKE_OSX_ARCHITECTURES MATCHES "arm64")
                set(VPX_TOOLCHAIN_FLAG --target=arm64-darwin24-gcc)
            endif()
        endif()
    endif()

    ExternalProject_Add(${VPX_TARGET}
        GIT_REPOSITORY  ${DEP_VPX_GIT_REPO}
        GIT_TAG         ${DEP_VPX_GIT_TAG}
        GIT_PROGRESS    OFF
        PREFIX          ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    0
        UPDATE_COMMAND  ""
        INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${VPX_APPLE_LDFLAGS_ENV_CMD}
            <SOURCE_DIR>/configure
                --prefix=<INSTALL_DIR>
                --enable-shared
                --disable-static
                --disable-examples
                ${VPX_TOOLCHAIN_FLAG}
        BUILD_COMMAND make -j${AV_BUILD_DEPENDENCIES_PARALLEL}
    )
    av_register_dep(${VPX_TARGET})
endif()
