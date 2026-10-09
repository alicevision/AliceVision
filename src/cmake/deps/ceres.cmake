# =============================================================================
# deps/ceres.cmake
# Dependencies: eigen, suitesparse, lapack
# Provides:     CERES_TARGET, CERES_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_CERES)
    set(CERES_TARGET ceres)

    av_add_cmake_dep(
        TARGET         ${CERES_TARGET}
        SOURCE_DIR     ceres-solver
        GIT_REPOSITORY ${DEP_CERES_GIT_REPO}
        GIT_TAG        ${DEP_CERES_GIT_TAG}
        EXTRA_CMAKE_FLAGS
            ${SUITESPARSE_CMAKE_FLAGS}
            ${EIGEN_CMAKE_FLAGS}
            ${LAPACK_CMAKE_FLAGS}
            # Since Ceres 2.3 the options are prefixed with WITH_, and SuiteSparse is opt-in
            -DWITH_ACCELERATESPARSE:BOOL=$<IF:$<PLATFORM_ID:Darwin>,ON,OFF>
            -DWITH_SUITESPARSE:BOOL=$<IF:$<PLATFORM_ID:Darwin>,OFF,ON>
            -DWITH_LAPACK:BOOL=ON
            -DWITH_CUDA=OFF  # CUDA solvers unused by AliceVision; drags cuBLAS/cuSOLVER/cuSPARSE
            -DBUILD_EXAMPLES:BOOL=OFF
            -DBUILD_TESTING:BOOL=OFF
        DEPENDS ${EIGEN_TARGET} ${SUITESPARSE_TARGET} ${LAPACK_TARGET}
    )

    set(CERES_CMAKE_FLAGS
        ${SUITESPARSE_CMAKE_FLAGS}
        -DCeres_DIR=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/Ceres
    )
endif()
