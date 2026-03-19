# =============================================================================
# deps/osi.cmake
# Dependencies: coinutils
# Provides:     OSI_TARGET, OSI_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_OSI)
    set(OSI_TARGET osi)

    av_add_cmake_dep(
        TARGET         ${OSI_TARGET}
        SOURCE_DIR     osi
        GIT_REPOSITORY ${DEP_OSI_GIT_REPO}
        GIT_TAG        ${DEP_OSI_GIT_TAG}
        DEPENDS        ${COINUTILS_TARGET}
        EXTRA_CMAKE_FLAGS
            -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=TRUE
    )

    set(OSI_CMAKE_FLAGS -DOsi_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/osi)
endif()
