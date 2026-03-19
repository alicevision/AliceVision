# =============================================================================
# deps/lapack.cmake
# Dependencies: tbb
# Provides:     LAPACK_TARGET, LAPACK_CMAKE_FLAGS, BLAS_LIBRARIES, LAPACK_LIBRARIES
# =============================================================================

if(AV_BUILD_LAPACK)
    set(LAPACK_TARGET lapack)

    av_add_cmake_dep(
        TARGET     ${LAPACK_TARGET}
        SOURCE_DIR lapack
        URL        ${DEP_LAPACK_URL}
        URL_HASH   ${DEP_LAPACK_HASH}
        EXTRA_CMAKE_FLAGS
            -DBUILD_SHARED_LIBS:BOOL=TRUE
        DEPENDS    ${TBB_TARGET}
    )

    set(BLAS_LIBRARIES
        ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libblas${CMAKE_SHARED_LIBRARY_SUFFIX}
    )
    set(LAPACK_LIBRARIES
        ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/liblapack${CMAKE_SHARED_LIBRARY_SUFFIX}
    )
    set(LAPACK_CMAKE_FLAGS
        "-DBLAS_LIBRARIES=${BLAS_LIBRARIES} -DLAPACK_LIBRARIES=${LAPACK_LIBRARIES}"
    )

endif()
