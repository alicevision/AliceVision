# =============================================================================
# deps/core.cmake — Core utilities and foundational libraries
#
# Dependencies: (none — these are the roots of the dependency graph)
# Provides:     ZLIB_CMAKE_FLAGS, TBB_CMAKE_FLAGS, EIGEN_CMAKE_FLAGS,
#               BOOST_CMAKE_FLAGS, SWIG_CMAKE_FLAGS
# =============================================================================

# ── zlib ──────────────────────────────────────────────────────────────────────

if(AV_BUILD_ZLIB)
    set(ZLIB_TARGET zlib)

    av_add_cmake_dep(
        TARGET     ${ZLIB_TARGET}
        SOURCE_DIR zlib
        URL        ${DEP_ZLIB_URL}
        URL_HASH   ${DEP_ZLIB_HASH}
    )

    set(ZLIB_CMAKE_FLAGS -DZLIB_ROOT=${CMAKE_INSTALL_PREFIX})
endif()

# ── tbb ───────────────────────────────────────────────────────────────────────

if(AV_BUILD_TBB)
    set(TBB_TARGET tbb)

    av_add_cmake_dep(
        TARGET     ${TBB_TARGET}
        SOURCE_DIR tbb
        URL        ${DEP_TBB_URL}
        URL_HASH   ${DEP_TBB_HASH}
        EXTRA_CMAKE_FLAGS
            -DTBB_TEST:BOOL=OFF
            -DTBB_STRICT:BOOL=OFF
    )

    set(TBB_CMAKE_FLAGS -DTBB_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/TBB)
endif()

# ── eigen ─────────────────────────────────────────────────────────────────────

if(AV_BUILD_EIGEN)
    set(EIGEN_TARGET eigen)

    # Eigen only needs -DCMAKE_CXX_STANDARD and optional alignment flags —
    # it does not use a C compiler or build-type, so we bypass CMAKE_CORE_BUILD_FLAGS.
    ExternalProject_Add(${EIGEN_TARGET}
        URL              ${DEP_EIGEN_URL}
        URL_HASH         ${DEP_EIGEN_HASH}
        DOWNLOAD_DIR     ${BUILD_DIR}/download/eigen
        DOWNLOAD_NO_PROGRESS TRUE
        PREFIX           ${BUILD_DIR}
        BUILD_IN_SOURCE  0
        BUILD_ALWAYS     0
        UPDATE_COMMAND   ""
        SOURCE_DIR       ${CMAKE_CURRENT_BINARY_DIR}/eigen
        BINARY_DIR       ${BUILD_DIR}/eigen_build
        INSTALL_DIR      ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND}
            -DCMAKE_CXX_STANDARD=20
            ${EIGEN_CMAKE_ALIGNMENT_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
    )

    set(EIGEN_CMAKE_FLAGS
        ${EIGEN_CMAKE_ALIGNMENT_FLAGS}
        -DEigen3_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/eigen3/cmake
        -DEIGEN3_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include/eigen3
        -DEIGEN_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include/eigen3
        -DEigen_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include/eigen3
    )

    av_register_dep(${EIGEN_TARGET})
endif()

# ── expat ─────────────────────────────────────────────────────────────────────

if(AV_BUILD_EXPAT)
    set(EXPAT_TARGET expat)

    # NOTE: libexpat places its CMakeLists.txt one level below the repo root,
    # inside the "expat/" subdirectory — hence the explicit CONFIGURE_COMMAND.
    ExternalProject_Add(${EXPAT_TARGET}
        GIT_REPOSITORY  ${DEP_EXPAT_GIT_REPO}
        GIT_TAG         ${DEP_EXPAT_GIT_TAG}
        PREFIX          ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    0
        UPDATE_COMMAND  ""
        SOURCE_DIR      ${CMAKE_CURRENT_BINARY_DIR}/expat
        BINARY_DIR      ${BUILD_DIR}/libexpat_build
        INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND}
            ${CMAKE_CORE_BUILD_FLAGS}
            -DCMAKE_POLICY_VERSION_MINIMUM=3.5
            -DEXPAT_BUILD_DOCS:BOOL=OFF
            -DEXPAT_BUILD_EXAMPLES:BOOL=OFF
            -DEXPAT_BUILD_TOOLS:BOOL=OFF
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>/expat
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
    )

    av_register_dep(${EXPAT_TARGET})
endif()

# ── boost ─────────────────────────────────────────────────────────────────────

if(AV_BUILD_BOOST)
    set(BOOST_TARGET boost)

    if(WIN32)
        set(_boost_bootstrap_ext bat)
    else()
        set(_boost_bootstrap_ext sh)
    endif()

    # Boost uses its own b2 build system, not CMake.
    ExternalProject_Add(${BOOST_TARGET}
        URL              ${DEP_BOOST_URL}
        URL_HASH         ${DEP_BOOST_HASH}
        DOWNLOAD_DIR     ${BUILD_DIR}/download/boost
        DOWNLOAD_NO_PROGRESS TRUE
        PREFIX           ${BUILD_DIR}
        BUILD_IN_SOURCE  0
        BUILD_ALWAYS     0
        UPDATE_COMMAND   ""
        SOURCE_DIR       ${CMAKE_CURRENT_BINARY_DIR}/boost
        BINARY_DIR       ${BUILD_DIR}/boost_build
        INSTALL_DIR      ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            cd <SOURCE_DIR> &&
            ./bootstrap.${_boost_bootstrap_ext}
                --prefix=<INSTALL_DIR>
                --with-libraries=atomic,container,date_time,exception,graph,iostreams,json,log,math,program_options,regex,serialization,system,test,thread,stacktrace,timer
        BUILD_COMMAND
            cd <SOURCE_DIR> &&
            ./b2 --prefix=<INSTALL_DIR>
                variant=${DEPS_CMAKE_BUILD_TYPE_LOWERCASE}
                cxxstd=20 link=shared threading=multi
                -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        INSTALL_COMMAND
            cd <SOURCE_DIR> &&
            ./b2
                variant=${DEPS_CMAKE_BUILD_TYPE_LOWERCASE}
                cxxstd=20 link=shared threading=multi
                install
        DEPENDS ${ZLIB_TARGET}
    )

    set(BOOST_CMAKE_FLAGS -DBOOST_ROOT=${CMAKE_INSTALL_PREFIX})
    av_register_dep(${BOOST_TARGET})
    unset(_boost_bootstrap_ext)
endif()

# ── pybind11 ──────────────────────────────────────────────────────────────────

if(AV_BUILD_PYBIND11)
    set(PYBIND11_TARGET pybind11)

    av_add_cmake_dep(
        TARGET         ${PYBIND11_TARGET}
        SOURCE_DIR     pybind11
        GIT_REPOSITORY ${DEP_PYBIND11_GIT_REPO}
        GIT_TAG        ${DEP_PYBIND11_GIT_TAG}
        EXTRA_CMAKE_FLAGS
            -DPython_EXECUTABLE=${Python_EXECUTABLE}
    )
endif()

# ── swig ──────────────────────────────────────────────────────────────────────

if(AV_BUILD_SWIG)
    set(SWIG_TARGET SWIG)

    av_add_cmake_dep(
        TARGET         ${SWIG_TARGET}
        SOURCE_DIR     SWIG
        GIT_REPOSITORY ${DEP_SWIG_GIT_REPO}
        GIT_TAG        ${DEP_SWIG_GIT_TAG}
    )

    set(SWIG_CMAKE_FLAGS
        -DSWIG_DIR=${CMAKE_INSTALL_PREFIX}/share/swig/${DEP_SWIG_VERSION}
        -DSWIG_EXECUTABLE=${CMAKE_INSTALL_PREFIX}/bin-deps
    )
endif()
