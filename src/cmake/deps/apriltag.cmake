# =============================================================================
# deps/apriltag.cmake
# Dependencies: (none)
# Provides:     APRILTAG_TARGET, APRILTAG_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_APRILTAG)
    set(APRILTAG_TARGET apriltag)

    av_add_cmake_dep(
        TARGET         ${APRILTAG_TARGET}
        SOURCE_DIR     apriltag
        GIT_REPOSITORY ${DEP_APRILTAG_GIT_REPO}
        GIT_TAG        ${DEP_APRILTAG_GIT_TAG}
        EXTRA_CMAKE_FLAGS
            -DBUILD_PYTHON_WRAPPER=OFF
            -DOpenCV_FOUND=OFF
    )

    set(APRILTAG_CMAKE_FLAGS
        -Dapriltag_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/apriltag/cmake
    )
endif()
