# =============================================================================
# deps/geogram.cmake
# Dependencies: zlib
# Provides:     GEOGRAM_TARGET, GEOGRAM_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_GEOGRAM)
    if(WIN32)
        set(_geo_platform -DVORPALINE_PLATFORM=Win-vs-dynamic-generic)
    elseif(APPLE)
        if(CMAKE_OSX_ARCHITECTURES MATCHES "x86_64")
            set(_geo_platform -DVORPALINE_PLATFORM=Darwin-clang-dynamic)
        elseif(CMAKE_OSX_ARCHITECTURES MATCHES "aarch64|arm64")
            set(_geo_platform -DVORPALINE_PLATFORM=Darwin-aarch64-clang-dynamic)
        else()
            message(FATAL_ERROR
                "Geogram: unsupported CMAKE_OSX_ARCHITECTURES processor '${CMAKE_OSX_ARCHITECTURES}'. "
                "Expected x86_64 or aarch64/arm64.")
        endif()
    elseif(UNIX)
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
            set(_geo_platform -DVORPALINE_PLATFORM=Linux64-gcc-dynamic)
        elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
            set(_geo_platform
                -DVORPALINE_PLATFORM=Linux64-gcc-aarch64
                -DVORPALINE_BUILD_DYNAMIC=ON
            )
        else()
            message(FATAL_ERROR
                "Geogram: unsupported Linux processor '${CMAKE_SYSTEM_PROCESSOR}'. "
                "Expected x86_64 or aarch64/arm64.")
        endif()
    endif()

    set(GEOGRAM_TARGET geogram)

    av_add_cmake_dep(
        TARGET     ${GEOGRAM_TARGET}
        SOURCE_DIR geogram
        URL        ${DEP_GEOGRAM_URL}
        URL_HASH   ${DEP_GEOGRAM_HASH}
        EXTRA_CMAKE_FLAGS
            ${_geo_platform}
            ${ZLIB_CMAKE_FLAGS}
            -DGEOGRAM_WITH_HLBFGS=OFF
            -DGEOGRAM_WITH_TETGEN=OFF
            -DGEOGRAM_WITH_GRAPHICS=OFF
            -DGEOGRAM_WITH_EXPLORAGRAM=OFF
            -DGEOGRAM_WITH_LUA=OFF
        DEPENDS ${ZLIB_TARGET}
    )

    set(GEOGRAM_CMAKE_FLAGS
        -DGEOGRAM_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
        -DGEOGRAM_INCLUDE_DIR=${CMAKE_INSTALL_PREFIX}/include/geogram1
    )
    unset(_geo_platform)
endif()
