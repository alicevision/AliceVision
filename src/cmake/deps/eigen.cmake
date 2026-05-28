# =============================================================================
# deps/eigen.cmake
# Dependencies: (none)
# Provides:     EIGEN_TARGET, EIGEN_CMAKE_FLAGS
#
# NOTE: Eigen only needs -DCMAKE_CXX_STANDARD and optional alignment flags —
# it does not use a C compiler or build-type, so we bypass CMAKE_CORE_BUILD_FLAGS.
# =============================================================================

if(AV_BUILD_EIGEN)
    set(EIGEN_TARGET eigen)

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
            ${OPENMP_CMAKE_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            -DPKGCONFIG_INSTALL_DIR:PATH=<INSTALL_DIR>/${CMAKE_INSTALL_LIBDIR}/pkgconfig
            <SOURCE_DIR>
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${OPENMP_TARGET}
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
