# =============================================================================
# deps/geometry.cmake — 3D geometry and scene I/O libraries
#
# Dependencies: zlib, boost, openexr, eigen, tbb, flann, lz4, cuda (optional)
# Provides:     GEOGRAM_CMAKE_FLAGS, ASSIMP_CMAKE_FLAGS, ALEMBIC_CMAKE_FLAGS,
#               E57FORMAT_CMAKE_FLAGS, OPENMESH_CMAKE_FLAGS, PCL_CMAKE_FLAGS,
#               USD_CMAKE_FLAGS
# =============================================================================

# ── Geogram ───────────────────────────────────────────────────────────────────

if(AV_BUILD_GEOGRAM)
    # Geogram requires a platform string that encodes OS + compiler + link mode.
    if(WIN32)
        set(_geo_platform -DVORPALINE_PLATFORM=Win-vs-dynamic-generic)
    elseif(APPLE)
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
            set(_geo_platform -DVORPALINE_PLATFORM=Darwin-clang-dynamic)
        elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
            set(_geo_platform -DVORPALINE_PLATFORM=Darwin-aarch64-clang-dynamic)
        else()
            message(FATAL_ERROR
                "Geogram: unsupported Apple processor '${CMAKE_SYSTEM_PROCESSOR}'. "
                "Expected x86_64 or aarch64/arm64.")
        endif()
    elseif(UNIX)
        if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
            set(_geo_platform -DVORPALINE_PLATFORM=Linux64-gcc-dynamic)
        elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
            # Dynamic build must be requested explicitly on Linux aarch64
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

# ── Assimp ────────────────────────────────────────────────────────────────────

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
        -DAssimp_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/assimp-${DEP_ASSIMP_VERSION}
    )
endif()

# ── Alembic ───────────────────────────────────────────────────────────────────

if(AV_BUILD_ALEMBIC)
    set(ALEMBIC_TARGET alembic)

    av_add_cmake_dep(
        TARGET     ${ALEMBIC_TARGET}
        SOURCE_DIR alembic
        URL        ${DEP_ALEMBIC_URL}
        URL_HASH   ${DEP_ALEMBIC_HASH}
        EXTRA_CMAKE_FLAGS
            ${ZLIB_CMAKE_FLAGS}
            ${ILMBASE_CMAKE_FLAGS}
            -DUSE_TESTS=OFF
        DEPENDS ${BOOST_TARGET} ${OPENEXR_TARGET} ${ZLIB_TARGET}
    )

    set(ALEMBIC_CMAKE_FLAGS
        -DAlembic_DIR:PATH=${CMAKE_INSTALL_PREFIX}/lib/cmake/Alembic
    )
endif()

# ── libE57Format ──────────────────────────────────────────────────────────────

if(AV_BUILD_E57FORMAT)
    set(E57FORMAT_TARGET E57Format)

    av_add_cmake_dep(
        TARGET         ${E57FORMAT_TARGET}
        SOURCE_DIR     E57Format
        GIT_REPOSITORY ${DEP_E57FORMAT_GIT_REPO}
        GIT_TAG        ${DEP_E57FORMAT_GIT_TAG}
        EXTRA_CMAKE_FLAGS
            -DE57_BUILD_TEST:BOOL=OFF
            -DBUILD_SHARED_LIBS:BOOL=ON
            -DCMAKE_CXX_STANDARD=17
    )

    set(E57FORMAT_CMAKE_FLAGS
        -DE57FORMAT_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/E57Format
    )
endif()

# ── OpenMesh ──────────────────────────────────────────────────────────────────

if(AV_BUILD_OPENMESH)
    set(OPENMESH_TARGET OpenMesh)

    # OpenMesh always builds in Release regardless of the parent build type.
    ExternalProject_Add(${OPENMESH_TARGET}
        URL              ${DEP_OPENMESH_URL}
        URL_HASH         ${DEP_OPENMESH_HASH}
        DOWNLOAD_DIR     ${BUILD_DIR}/download/OpenMesh
        DOWNLOAD_NO_PROGRESS TRUE
        PREFIX           ${BUILD_DIR}
        BUILD_IN_SOURCE  0
        BUILD_ALWAYS     0
        UPDATE_COMMAND   ""
        SOURCE_DIR       ${CMAKE_CURRENT_BINARY_DIR}/OpenMesh
        BINARY_DIR       ${BUILD_DIR}/OpenMesh_build
        INSTALL_DIR      ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND}
            -DCMAKE_BUILD_TYPE=Release
            -DBUILD_APPS=OFF
            -DOPENMESH_DOCS=OFF
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            -DCMAKE_POLICY_VERSION_MINIMUM=3.5
            <SOURCE_DIR>
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config Release
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
    )

    set(OPENMESH_CMAKE_FLAGS
        -DOPENMESH_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/OpenMesh/cmake
    )
    av_register_dep(${OPENMESH_TARGET})
endif()

# ── PCL (Point Cloud Library) — optional ─────────────────────────────────────

if(AV_BUILD_PCL)
    set(PCL_TARGET pcl)

    av_add_cmake_dep(
        TARGET     ${PCL_TARGET}
        SOURCE_DIR pcl
        URL        ${DEP_PCL_URL}
        URL_HASH   ${DEP_PCL_HASH}
        EXTRA_CMAKE_FLAGS
            ${EIGEN_CMAKE_FLAGS}
            ${BOOST_CMAKE_FLAGS}
            ${PNG_CMAKE_FLAGS}
            ${CUDA_CMAKE_FLAGS}
            ${FLANN_CMAKE_FLAGS}
            ${LZ4_CMAKE_FLAGS}
            ${ZLIB_CMAKE_FLAGS}
            -DWITH_CUDA:BOOL=${AV_USE_CUDA}
            -DWITH_OPENGL:BOOL=OFF
            -DWITH_OPENMP:BOOL=ON
            -DWITH_LIBUSB:BOOL=OFF
            -DWITH_VTK:BOOL=OFF
            -DWITH_PCAP:BOOL=OFF
        DEPENDS
            ${FLANN_TARGET} ${LZ4_TARGET}
            ${EIGEN_TARGET} ${BOOST_TARGET}
            ${PNG_TARGET}   ${CUDA_TARGET}
            ${ZLIB_TARGET}
    )

    set(PCL_CMAKE_FLAGS -DPCL_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/pcl-1.12/)
endif()

# ── USD (Pixar Universal Scene Description) — optional ───────────────────────

if(AV_BUILD_USD)
    set(USD_TARGET pxr)

    # USD uses its own Python build script, not CMake configure.
    ExternalProject_Add(${USD_TARGET}
        GIT_REPOSITORY  ${DEP_USD_GIT_REPO}
        GIT_TAG         ${DEP_USD_GIT_TAG}
        PREFIX          ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    0
        UPDATE_COMMAND  ""
        CONFIGURE_COMMAND ""
        INSTALL_COMMAND   ""
        SOURCE_DIR        ${CMAKE_CURRENT_BINARY_DIR}/usd
        BINARY_DIR        ${BUILD_DIR}/usd_build
        INSTALL_DIR       ${CMAKE_INSTALL_PREFIX}
        BUILD_COMMAND
            python ${CMAKE_CURRENT_BINARY_DIR}/usd/build_scripts/build_usd.py
                --build-shared
                --no-examples --no-tools --no-ptex --no-prman
                --no-openimageio --no-opencolorio --no-alembic
                --no-draco --no-materialx
                --no-tutorials --no-tests --no-docs --no-python
                <INSTALL_DIR>
    )

    set(USD_CMAKE_FLAGS -Dpxr_DIR:PATH=${CMAKE_INSTALL_PREFIX})
    av_register_dep(${USD_TARGET})
endif()
