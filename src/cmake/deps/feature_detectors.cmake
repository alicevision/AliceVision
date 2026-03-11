# =============================================================================
# deps/feature_detectors.cmake — Image feature detection libraries
#
# Dependencies: boost, cuda (optional), opencv, eigen, tbb
# Provides:     POPSIFT_CMAKE_FLAGS, CCTAG_CMAKE_FLAGS, APRILTAG_CMAKE_FLAGS
# =============================================================================

# ── PopSift (GPU SIFT) ────────────────────────────────────────────────────────

if(AV_BUILD_POPSIFT)
    set(POPSIFT_TARGET popsift)

    av_add_cmake_dep(
        TARGET         ${POPSIFT_TARGET}
        SOURCE_DIR     popsift
        GIT_REPOSITORY ${DEP_POPSIFT_GIT_REPO}
        GIT_TAG        ${DEP_POPSIFT_GIT_TAG}
        EXTRA_CMAKE_FLAGS
            ${BOOST_CMAKE_FLAGS}
            ${CUDA_CMAKE_FLAGS}
            -DPopSift_BUILD_EXAMPLES:BOOL=OFF
        DEPENDS ${BOOST_TARGET} ${CUDA_TARGET}
    )

    set(POPSIFT_CMAKE_FLAGS
        -DPopSift_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/PopSift
    )
endif()

# ── CCTag ─────────────────────────────────────────────────────────────────────

if(AV_BUILD_CCTAG)
    set(CCTAG_TARGET cctag)

    av_add_cmake_dep(
        TARGET         ${CCTAG_TARGET}
        SOURCE_DIR     cctag
        GIT_REPOSITORY ${DEP_CCTAG_GIT_REPO}
        GIT_TAG        ${DEP_CCTAG_GIT_TAG}
        EXTRA_CMAKE_FLAGS
            ${BOOST_CMAKE_FLAGS}
            ${CUDA_CMAKE_FLAGS}
            ${OPENCV_CMAKE_FLAGS}
            ${EIGEN_CMAKE_FLAGS}
            ${TBB_CMAKE_FLAGS}
            -DCCTAG_WITH_CUDA:BOOL=${AV_USE_CUDA}
            -DCCTAG_BUILD_TESTS=OFF
            -DCCTAG_BUILD_APPS=OFF
            -DCCTAG_EIGEN_MEMORY_ALIGNMENT=ON
            -DCCTAG_CXX_STANDARD=20
        DEPENDS
            ${BOOST_TARGET} ${CUDA_TARGET}
            ${OPENCV_TARGET} ${EIGEN_TARGET} ${TBB_TARGET}
    )

    set(CCTAG_CMAKE_FLAGS
        -DCCTag_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/CCTag
    )
endif()

# ── AprilTag ──────────────────────────────────────────────────────────────────

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
