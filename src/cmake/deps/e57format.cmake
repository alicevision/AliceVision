# =============================================================================
# deps/e57format.cmake
# Dependencies: (none)
# Provides:     E57FORMAT_TARGET, E57FORMAT_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_E57FORMAT)
    set(E57FORMAT_TARGET E57Format)

    av_add_cmake_dep(
        TARGET         ${E57FORMAT_TARGET}
        SOURCE_DIR     E57Format
        GIT_REPOSITORY ${DEP_E57FORMAT_GIT_REPO}
        GIT_TAG        ${DEP_E57FORMAT_GIT_TAG}
        EXTRA_CMAKE_FLAGS
            ${XERCESC_CMAKE_FLAGS}
            -DE57_BUILD_TEST:BOOL=OFF
            -DBUILD_SHARED_LIBS:BOOL=ON
        DEPENDS ${XERCESC_TARGET}
    )

    set(E57FORMAT_CMAKE_FLAGS
        -DE57FORMAT_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/E57Format
    )
endif()
