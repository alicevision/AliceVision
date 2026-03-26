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
            <SOURCE_DIR>/configure
                --prefix=<INSTALL_DIR>
                --enable-shared
                --disable-static
                --disable-examples
        BUILD_COMMAND make -j${AV_BUILD_DEPENDENCIES_PARALLEL}
    )
    av_register_dep(${VPX_TARGET})
endif()
