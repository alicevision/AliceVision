# =============================================================================
# deps/popsift.cmake
# Dependencies: boost, cuda (optional)
# Provides:     POPSIFT_TARGET, POPSIFT_CMAKE_FLAGS
# =============================================================================

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
        -DPopSift_DIR:PATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/PopSift
    )
endif()
