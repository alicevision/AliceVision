# =============================================================================
# deps/assimp.cmake
# Dependencies: zlib
# Provides:     ASSIMP_TARGET, ASSIMP_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_ASSIMP)
    set(ASSIMP_TARGET assimp)

    av_add_cmake_dep(
        TARGET     ${ASSIMP_TARGET}
        SOURCE_DIR assimp
        URL        ${DEP_ASSIMP_URL}
        URL_HASH   ${DEP_ASSIMP_HASH}
        EXTRA_CMAKE_FLAGS
            -DASSIMP_BUILD_ASSIMP_TOOLS:BOOL=OFF
            -DASSIMP_BUILD_TESTS:BOOL=OFF
            -DASSIMP_BUILD_DRACO:BOOL=ON
            -DASSIMP_WARNINGS_AS_ERRORS=OFF
            ${ZLIB_CMAKE_FLAGS}
        DEPENDS ${ZLIB_TARGET}
    )

    set(ASSIMP_CMAKE_FLAGS
        -DAssimp_DIR:PATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/assimp-${DEP_ASSIMP_VERSION}
    )
endif()
