# =============================================================================
# deps/png.cmake
# Dependencies: zlib
# Provides:     PNG_TARGET, PNG_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_PNG)
    set(PNG_TARGET png)

    if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
        set(_png_neon on)
    else()
        set(_png_neon off)
    endif()

    av_add_cmake_dep(
        TARGET     ${PNG_TARGET}
        SOURCE_DIR png
        URL        ${DEP_PNG_URL}
        URL_HASH   ${DEP_PNG_HASH}
        EXTRA_CMAKE_FLAGS
            ${ZLIB_CMAKE_FLAGS}
            -DPNG_ARM_NEON=${_png_neon}
            -DPNG_STATIC:BOOL=OFF
            -DPNG_EXECUTABLES:BOOL=OFF
        DEPENDS    ${ZLIB_TARGET}
    )

    unset(_png_neon)

    set(PNG_CMAKE_FLAGS
        -DPNG_LIBRARY=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/libpng${CMAKE_SHARED_LIBRARY_SUFFIX}
        -DPNG_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include
    )

endif()
