# =============================================================================
# deps/coinutils.cmake
# Dependencies: (none)
# Provides:     COINUTILS_TARGET, COINUTILS_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_COINUTILS)
    set(COINUTILS_TARGET coinutils)

    av_add_cmake_dep(
        TARGET         ${COINUTILS_TARGET}
        SOURCE_DIR     coinutils
        GIT_REPOSITORY ${DEP_COINUTILS_GIT_REPO}
        GIT_TAG        ${DEP_COINUTILS_GIT_TAG}
        EXTRA_CMAKE_FLAGS
            -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=TRUE
    )

    set(COINUTILS_CMAKE_FLAGS
        -DCoinUtils_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/coinutils
    )
endif()
