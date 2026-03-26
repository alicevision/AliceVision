# =============================================================================
# deps/suitesparse.cmake
# Dependencies: lapack (BLAS_LIBRARIES, LAPACK_LIBRARIES), cuda (optionnel)
# Provides:     SUITESPARSE_TARGET, SUITESPARSE_CMAKE_FLAGS
#
# NOTE: Requires gmp and mpfr as prerequisites, built inline here since they
# exist solely to support SuiteSparse and are not exposed as standalone options.
# SuiteSparse uses its own Makefile system — make is the only build tool.
# =============================================================================

if(AV_BUILD_SUITESPARSE)

    # ── GMP ───────────────────────────────────────────────────────────────────

    ExternalProject_Add(gmp
        URL              ${DEP_GMP_URL}
        URL_HASH         ${DEP_GMP_HASH}
        DOWNLOAD_DIR     ${BUILD_DIR}/download/gmp
        DOWNLOAD_NO_PROGRESS TRUE
        PREFIX           ${BUILD_DIR}
        BUILD_IN_SOURCE  0
        BUILD_ALWAYS     0
        UPDATE_COMMAND   ""
        INSTALL_DIR      ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            <SOURCE_DIR>/configure
                --prefix=<INSTALL_DIR>
                --enable-cxx
                --enable-static=no
        BUILD_COMMAND make -j${AV_BUILD_DEPENDENCIES_PARALLEL}
    )
    av_register_dep(gmp)

    # ── MPFR ──────────────────────────────────────────────────────────────────

    ExternalProject_Add(mpfr
        URL              ${DEP_MPFR_URL}
        URL_HASH         ${DEP_MPFR_HASH}
        DOWNLOAD_DIR     ${BUILD_DIR}/download/mpfr
        DOWNLOAD_NO_PROGRESS TRUE
        PREFIX           ${BUILD_DIR}
        BUILD_IN_SOURCE  0
        BUILD_ALWAYS     0
        UPDATE_COMMAND   ""
        INSTALL_DIR      ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            autoreconf -f -i <SOURCE_DIR> &&
            <SOURCE_DIR>/configure
                --prefix=<INSTALL_DIR>
                --with-gmp=<INSTALL_DIR>
                --enable-static=no
        BUILD_COMMAND make -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS gmp
    )
    av_register_dep(mpfr)

    # ── SuiteSparse ───────────────────────────────────────────────────────────

    set(SUITESPARSE_TARGET suitesparse)

    #if(APPLE)
    #    set(_ss_env
    #        VERBOSE=1
    #        MPFR_ROOT=${CMAKE_INSTALL_PREFIX}
    #        GMP_ROOT=${CMAKE_INSTALL_PREFIX}
    #        DYLD_LIBRARY_PATH=${DYLD_LIBRARY_PATH}:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}
    #    )
    #    set(_ss_blas
    #        BLAS="${BLAS_LIBRARIES}"
    #        LAPACK="${LAPACK_LIBRARIES}"
    #        LAPACK_LIBRARIES="${LAPACK_LIBRARIES}"
    #    )
    #else()
    #    set(_ss_env
    #        VERBOSE=1
    #        MPFR_ROOT=${CMAKE_INSTALL_PREFIX}
    #        GMP_ROOT=${CMAKE_INSTALL_PREFIX}
    #        LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}
    #    )
    #    set(_ss_blas
    #        BLAS_LIBRARIES="${BLAS_LIBRARIES}"
    #        BLAS="${BLAS_LIBRARIES}"
    #        LAPACK="${LAPACK_LIBRARIES}"
    #        LAPACK_LIBRARIES="${LAPACK_LIBRARIES}"
    #    )
    #endif()

    if(AV_USE_CUDA)
        set(_cuda_cmake_flags "-DENABLE_CUDA=ON\ -DCMAKE_CXX_FLAGS=-I${CUDA_TOOLKIT_ROOT_DIR}/include\ -DCMAKE_C_FLAGS=-I${CUDA_TOOLKIT_ROOT_DIR}/include")
    endif()

    ExternalProject_Add(${SUITESPARSE_TARGET}
        URL              ${DEP_SUITESPARSE_URL}
        URL_HASH         ${DEP_SUITESPARSE_HASH}
        DOWNLOAD_DIR     ${BUILD_DIR}/download/suitesparse
        DOWNLOAD_NO_PROGRESS TRUE
        PREFIX           ${BUILD_DIR}
        BUILD_IN_SOURCE  1
        BUILD_ALWAYS     0
        UPDATE_COMMAND   ""
        SOURCE_DIR       ${CMAKE_CURRENT_BINARY_DIR}/suitesparse
        #BINARY_DIR       ${CMAKE_CURRENT_BINARY_DIR}/suitesparse
        INSTALL_DIR      ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND ""
        BUILD_COMMAND
            $(MAKE) -j${AV_BUILD_DEPENDENCIES_PARALLEL}
            CMAKE_OPTIONS=${LAPACK_CMAKE_FLAGS}\ -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}\ -DCMAKE_INSTALL_LIBDIR=lib\ ${_cuda_cmake_flags}
        INSTALL_COMMAND
            $(MAKE) install
        DEPENDS ${LAPACK_TARGET} mpfr
    )

    set(SUITESPARSE_CMAKE_FLAGS
        ${LAPACK_CMAKE_FLAGS}
        -DSUITESPARSE_INCLUDE_DIR_HINTS=${CMAKE_INSTALL_PREFIX}/include
        -DSUITESPARSE_LIBRARY_DIR_HINTS=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}
    )

    av_register_dep(${SUITESPARSE_TARGET})

    unset(_cuda_cmake_flags)
    #unset(_ss_env)
    #unset(_ss_blas)
    #unset(_ss_cmake_opts)
endif()
