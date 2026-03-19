# =============================================================================
# deps/openexr.cmake
# Dependencies: zlib
# Provides:     OPENEXR_TARGET, OPENEXR_CMAKE_FLAGS, ILMBASE_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_OPENEXR)
    set(OPENEXR_TARGET openexr)

    av_add_cmake_dep(
        TARGET     ${OPENEXR_TARGET}
        SOURCE_DIR openexr
        URL        ${DEP_OPENEXR_URL}
        URL_HASH   ${DEP_OPENEXR_HASH}
        EXTRA_CMAKE_FLAGS
            #-DOPENEXR_BUILD_PYTHON_LIBS:BOOL=OFF
            -DOPENEXR_INSTALL_TOOLS:BOOL=OFF
            -DOPENEXR_INSTALL_EXAMPLES:BOOL=OFF
            -DBUILD_DOCS:BOOL=OFF
            -DBUILD_TESTING:BOOL=OFF
            -DOPENEXR_BUILD_TOOLS:BOOL=OFF
            ${ZLIB_CMAKE_FLAGS}
        DEPENDS ${ZLIB_TARGET}
    )

    set(ILMBASE_CMAKE_FLAGS
        -DILMBASE_ROOT=${CMAKE_INSTALL_PREFIX}
        -DILMBASE_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include/OpenEXR
    )
    set(OPENEXR_CMAKE_FLAGS
        ${ILMBASE_CMAKE_FLAGS}
        -DOPENEXR_ROOT=${CMAKE_INSTALL_PREFIX}
        -DOPENEXR_INCLUDE_PATH=${CMAKE_INSTALL_PREFIX}/include
    )
endif()
