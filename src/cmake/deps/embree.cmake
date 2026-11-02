# =============================================================================
# deps/embree.cmake
# Dependencies: tbb
# Provides:     EMBREE_TARGET, EMBREE_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_EMBREE)
    set(EMBREE_TARGET embree)

    av_add_cmake_dep(
        TARGET     ${EMBREE_TARGET}
        SOURCE_DIR embree
        URL        ${DEP_EMBREE_URL}
        URL_HASH   ${DEP_EMBREE_HASH}
        EXTRA_CMAKE_FLAGS
            ${TBB_CMAKE_FLAGS}
            -DEMBREE_TUTORIALS:BOOL=OFF
            -DEMBREE_ISPC_SUPPORT:BOOL=OFF
            -DEMBREE_TASKING_SYSTEM=TBB
        DEPENDS ${TBB_TARGET}
    )

    set(EMBREE_CMAKE_FLAGS
        -Dembree_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/embree-${DEP_EMBREE_VERSION}
    )
endif()
