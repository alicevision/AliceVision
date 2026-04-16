# =============================================================================
# deps/lemon.cmake
# Dependencies: (none)
# Provides:     LEMON_TARGET, LEMON_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_LEMON)
    set(LEMON_TARGET LEMON)

    av_add_cmake_dep(
        TARGET         ${LEMON_TARGET}
        SOURCE_DIR     LEMON
        GIT_REPOSITORY ${DEP_LEMON_GIT_REPO}
        GIT_TAG        ${DEP_LEMON_GIT_TAG}
        EXTRA_CMAKE_FLAGS
          -DBUILD_SHARED_LIBS=OFF
          -DCMAKE_CXX_STANDARD=14
    )

    set(LEMON_CMAKE_FLAGS
        -DLEMON_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/lemon/cmake
    )
endif()
