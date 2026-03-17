# =============================================================================
# deps/tbb.cmake
# Dependencies: (none)
# Provides:     TBB_TARGET, TBB_CMAKE_FLAGS
# =============================================================================

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

set(TBB_CMAKE_FLAGS -DTBB_DIR:PATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/TBB)
endif()
