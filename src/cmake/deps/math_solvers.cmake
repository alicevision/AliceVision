# =============================================================================
# deps/math_solvers.cmake — Linear algebra, sparse solvers and graph libraries
#
# Dependencies: tbb, eigen
# Provides:     LAPACK_CMAKE_FLAGS, SUITESPARSE_CMAKE_FLAGS, CERES_CMAKE_FLAGS,
#               LZ4_CMAKE_FLAGS, FLANN_CMAKE_FLAGS, NANOFLANN_CMAKE_FLAGS,
#               COINUTILS_CMAKE_FLAGS, OSI_CMAKE_FLAGS, CLP_CMAKE_FLAGS,
#               LEMON_CMAKE_FLAGS
# =============================================================================

# ── LAPACK / BLAS ─────────────────────────────────────────────────────────────

if(AV_BUILD_LAPACK)
    set(LAPACK_TARGET lapack)

    av_add_cmake_dep(
        TARGET     ${LAPACK_TARGET}
        SOURCE_DIR lapack
        URL        ${DEP_LAPACK_URL}
        URL_HASH   ${DEP_LAPACK_HASH}
        DEPENDS    ${TBB_TARGET}
    )

    set(BLAS_LIBRARIES
        ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libblas${CMAKE_SHARED_LIBRARY_SUFFIX}
    )
    set(LAPACK_LIBRARIES
        ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/liblapack${CMAKE_SHARED_LIBRARY_SUFFIX}
    )
    set(LAPACK_CMAKE_FLAGS
        -DBLAS_LIBRARIES=${BLAS_LIBRARIES}
        -DLAPACK_LIBRARIES=${LAPACK_LIBRARIES}
    )
endif()

# ── GMP + MPFR (SuiteSparse prerequisites) ────────────────────────────────────

if(AV_BUILD_SUITESPARSE)

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
            <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> --enable-cxx
        # GMP uses autoconf — make is the only build tool
        BUILD_COMMAND make -j${AV_BUILD_DEPENDENCIES_PARALLEL}
    )
    av_register_dep(gmp)

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
        # MPFR uses autoconf — make is the only build tool
        BUILD_COMMAND make -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS gmp
    )
    av_register_dep(mpfr)

    # ── SuiteSparse ───────────────────────────────────────────────────────────

    set(SUITESPARSE_TARGET suitesparse)

    # Build environment and BLAS/LAPACK flags differ between macOS and Linux
    if(APPLE)
        set(_ss_env
            VERBOSE=1
            MPFR_ROOT=${CMAKE_INSTALL_PREFIX}
            GMP_ROOT=${CMAKE_INSTALL_PREFIX}
            DYLD_LIBRARY_PATH=${DYLD_LIBRARY_PATH}:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}
        )
        set(_ss_blas
            BLAS="${BLAS_LIBRARIES}"
            LAPACK="${LAPACK_LIBRARIES}"
            LAPACK_LIBRARIES="${LAPACK_LIBRARIES}"
        )
    else()
        set(_ss_env
            VERBOSE=1
            MPFR_ROOT=${CMAKE_INSTALL_PREFIX}
            GMP_ROOT=${CMAKE_INSTALL_PREFIX}
            LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}
        )
        set(_ss_blas
            BLAS_LIBRARIES="${BLAS_LIBRARIES}"
            BLAS="${BLAS_LIBRARIES}"
            LAPACK="${LAPACK_LIBRARIES}"
            LAPACK_LIBRARIES="${LAPACK_LIBRARIES}"
        )
    endif()

    set(_ss_cmake_opts
        CMAKE_OPTIONS=-DBLAS_LIBRARIES=${BLAS_LIBRARIES}\ -DLAPACK_LIBRARIES=${LAPACK_LIBRARIES}\ -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    )

    ExternalProject_Add(${SUITESPARSE_TARGET}
        URL              ${DEP_SUITESPARSE_URL}
        URL_HASH         ${DEP_SUITESPARSE_HASH}
        DOWNLOAD_DIR     ${BUILD_DIR}/download/suitesparse
        DOWNLOAD_NO_PROGRESS TRUE
        PREFIX           ${BUILD_DIR}
        BUILD_IN_SOURCE  0
        BUILD_ALWAYS     0
        UPDATE_COMMAND   ""
        SOURCE_DIR       ${CMAKE_CURRENT_BINARY_DIR}/suitesparse
        BINARY_DIR       ${CMAKE_CURRENT_BINARY_DIR}/suitesparse
        INSTALL_DIR      ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND ""
        # SuiteSparse uses its own Makefile system — make is the only build tool
        BUILD_COMMAND
            cd <BINARY_DIR> &&
            ${_ss_env} make -j${AV_BUILD_DEPENDENCIES_PARALLEL}
            library
            CC=${CMAKE_C_COMPILER} CXX=${CMAKE_CXX_COMPILER}
            ${_ss_blas} ${_ss_cmake_opts}
        INSTALL_COMMAND
            cd <BINARY_DIR> &&
            ${_ss_env} make -j${AV_BUILD_DEPENDENCIES_PARALLEL}
            install library
            INSTALL=<INSTALL_DIR>
            CC=${CMAKE_C_COMPILER} CXX=${CMAKE_CXX_COMPILER}
            CMAKE_OPTIONS=-DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
        DEPENDS ${LAPACK_TARGET} mpfr
    )

    set(SUITESPARSE_CMAKE_FLAGS
        ${LAPACK_CMAKE_FLAGS}
        -DSUITESPARSE_INCLUDE_DIR_HINTS=${CMAKE_INSTALL_PREFIX}/include
        -DSUITESPARSE_LIBRARY_DIR_HINTS=${CMAKE_INSTALL_PREFIX}/lib
    )
    av_register_dep(${SUITESPARSE_TARGET})

    unset(_ss_env)
    unset(_ss_blas)
    unset(_ss_cmake_opts)
endif()

# ── Ceres ─────────────────────────────────────────────────────────────────────

if(AV_BUILD_CERES)
    set(CERES_TARGET ceres)

    av_add_cmake_dep(
        TARGET         ${CERES_TARGET}
        SOURCE_DIR     ceres-solver
        GIT_REPOSITORY ${DEP_CERES_GIT_REPO}
        GIT_TAG        ${DEP_CERES_GIT_TAG}
        EXTRA_CMAKE_FLAGS
            ${SUITESPARSE_CMAKE_FLAGS}
            -DSUITESPARSE:BOOL=ON
            -DLAPACK:BOOL=ON
            ${EIGEN_CMAKE_FLAGS}
            -DMINIGLOG=ON
            -DBUILD_EXAMPLES:BOOL=OFF
        DEPENDS ${EIGEN_TARGET} ${SUITESPARSE_TARGET}
    )

    set(CERES_CMAKE_FLAGS
        ${SUITESPARSE_CMAKE_FLAGS}
        -DCeres_DIR=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/Ceres
    )
endif()

# ── lz4 (required by FLANN) ───────────────────────────────────────────────────

if(AV_BUILD_FLANN)
    set(LZ4_TARGET lz4)

    # lz4 places its CMakeLists.txt under build/cmake/ — cannot use av_add_cmake_dep.
    ExternalProject_Add(${LZ4_TARGET}
        GIT_REPOSITORY  ${DEP_LZ4_GIT_REPO}
        GIT_TAG         ${DEP_LZ4_GIT_TAG}
        PREFIX          ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    0
        UPDATE_COMMAND  ""
        SOURCE_DIR      ${CMAKE_CURRENT_BINARY_DIR}/lz4
        BINARY_DIR      ${BUILD_DIR}/lz4_build
        INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND}
            ${CMAKE_CORE_BUILD_FLAGS}
            -DCMAKE_POLICY_VERSION_MINIMUM=3.5
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>/build/cmake/
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
        INSTALL_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --target install
    )

    set(LZ4_CMAKE_FLAGS -Dlz4_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/lz4/)
    av_register_dep(${LZ4_TARGET})

    # ── FLANN ─────────────────────────────────────────────────────────────────

    set(FLANN_TARGET flann)

    ExternalProject_Add(${FLANN_TARGET}
        GIT_REPOSITORY  ${DEP_FLANN_GIT_REPO}
        GIT_TAG         ${DEP_FLANN_GIT_TAG}
        PREFIX          ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    0
        UPDATE_COMMAND  ""
        SOURCE_DIR      ${CMAKE_CURRENT_BINARY_DIR}/flann
        BINARY_DIR      ${BUILD_DIR}/flann_build
        INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND} -E env
                PKG_CONFIG_PATH=${CMAKE_INSTALL_PREFIX}/lib64/pkgconfig/
            ${CMAKE_COMMAND}
            ${CMAKE_CORE_BUILD_FLAGS}
            -DBUILD_C_BINDINGS:BOOL=OFF
            -DBUILD_EXAMPLES=OFF
            -DBUILD_TESTS:BOOL=OFF
            -DBUILD_DOC:BOOL=OFF
            -DBUILD_PYTHON_BINDINGS:BOOL=OFF
            -DBUILD_MATLAB_BINDINGS:BOOL=OFF
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
        INSTALL_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --target install
        DEPENDS ${LZ4_TARGET}
    )

    set(FLANN_CMAKE_FLAGS -Dflann_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/flann/)
    av_register_dep(${FLANN_TARGET})
endif()

# ── NanoFLANN ─────────────────────────────────────────────────────────────────

if(AV_BUILD_NANOFLANN)
    set(NANOFLANN_TARGET nanoflann)

    ExternalProject_Add(${NANOFLANN_TARGET}
        GIT_REPOSITORY  ${DEP_NANOFLANN_GIT_REPO}
        GIT_TAG         ${DEP_NANOFLANN_GIT_TAG}
        PREFIX          ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    0
        UPDATE_COMMAND  ""
        SOURCE_DIR      ${CMAKE_CURRENT_BINARY_DIR}/nanoflann
        BINARY_DIR      ${BUILD_DIR}/nanoflann_build
        INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND} -E env
                PKG_CONFIG_PATH=${CMAKE_INSTALL_PREFIX}/lib64/pkgconfig/
            ${CMAKE_COMMAND}
            ${CMAKE_CORE_BUILD_FLAGS}
            -DCMAKE_POLICY_VERSION_MINIMUM=3.5
            -DNANOFLANN_BUILD_EXAMPLES=OFF
            -DNANOFLANN_BUILD_TESTS=OFF
            -DCMAKE_INSTALL_LIBDIR=lib
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
        INSTALL_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --target install
    )

    set(NANOFLANN_CMAKE_FLAGS
        -Dnanoflann_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/nanoflann/
    )
    av_register_dep(${NANOFLANN_TARGET})
endif()

# ── CoinUtils / Osi / Clp (LP solver stack) ───────────────────────────────────

if(AV_BUILD_COINUTILS)
    set(COINUTILS_TARGET coinutils)

    av_add_cmake_dep(
        TARGET         ${COINUTILS_TARGET}
        SOURCE_DIR     coinutils
        GIT_REPOSITORY ${DEP_COINUTILS_GIT_REPO}
        GIT_TAG        ${DEP_COINUTILS_GIT_TAG}
    )

    set(COINUTILS_CMAKE_FLAGS
        -DCoinUtils_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/coinutils
    )
endif()

if(AV_BUILD_OSI)
    set(OSI_TARGET osi)

    av_add_cmake_dep(
        TARGET         ${OSI_TARGET}
        SOURCE_DIR     osi
        GIT_REPOSITORY ${DEP_OSI_GIT_REPO}
        GIT_TAG        ${DEP_OSI_GIT_TAG}
        DEPENDS        ${COINUTILS_TARGET}
    )

    set(OSI_CMAKE_FLAGS -DOsi_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/osi)
endif()

if(AV_BUILD_CLP)
    set(CLP_TARGET clp)

    av_add_cmake_dep(
        TARGET         ${CLP_TARGET}
        SOURCE_DIR     clp
        GIT_REPOSITORY ${DEP_CLP_GIT_REPO}
        GIT_TAG        ${DEP_CLP_GIT_TAG}
        DEPENDS        ${COINUTILS_TARGET} ${OSI_TARGET}
    )

    set(CLP_CMAKE_FLAGS -DClp_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/clp)
endif()

# ── LEMON (graph library) ─────────────────────────────────────────────────────

if(AV_BUILD_LEMON)
    set(LEMON_TARGET LEMON)

    av_add_cmake_dep(
        TARGET         ${LEMON_TARGET}
        SOURCE_DIR     LEMON
        GIT_REPOSITORY ${DEP_LEMON_GIT_REPO}
        GIT_TAG        ${DEP_LEMON_GIT_TAG}
    )

    set(LEMON_CMAKE_FLAGS
        -DLEMON_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/lemon/cmake
    )
endif()
