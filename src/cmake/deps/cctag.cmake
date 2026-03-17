# =============================================================================
# deps/cctag.cmake
# Dependencies: boost, cuda (optional), opencv, eigen, tbb
# Provides:     CCTAG_TARGET, CCTAG_CMAKE_FLAGS
# =============================================================================

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
