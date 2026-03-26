# =============================================================================
# deps/libraw.cmake
# Dependencies: zlib
# Provides:     LIBRAW_TARGET, LIBRAW_CMAKE_FLAGS
#
# NOTE: Two-step build — fetch the unofficial CMake overlay first, then build
# the libraw sources using that overlay's CMakeLists.txt.
# =============================================================================

if(AV_BUILD_LIBRAW)
    set(LIBRAW_TARGET libraw)

    # Step 1 — fetch the CMake build-system overlay (no build/install)
    ExternalProject_Add(libraw_cmake
        GIT_REPOSITORY  ${DEP_LIBRAW_CMAKE_GIT_REPO}
        GIT_TAG         ${DEP_LIBRAW_CMAKE_GIT_TAG}
        PREFIX          ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    0
        UPDATE_COMMAND  ""
        SOURCE_DIR      ${CMAKE_CURRENT_BINARY_DIR}/libraw_cmake
        BINARY_DIR      ${BUILD_DIR}/libraw_cmake_build
        INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND ""
        BUILD_COMMAND     ""
        INSTALL_COMMAND   ""
    )

    # Step 2 — build libraw sources using the overlay CMakeLists.txt
    ExternalProject_Add(${LIBRAW_TARGET}
        GIT_REPOSITORY  ${DEP_LIBRAW_GIT_REPO}
        GIT_TAG         ${DEP_LIBRAW_GIT_TAG}
        PREFIX          ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    0
        UPDATE_COMMAND  ""
        SOURCE_DIR      ${CMAKE_CURRENT_BINARY_DIR}/libraw
        BINARY_DIR      ${CMAKE_CURRENT_BINARY_DIR}/libraw
        INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            cp <SOURCE_DIR>_cmake/CMakeLists.txt . &&
            cp -rf <SOURCE_DIR>_cmake/cmake . &&
            ${CMAKE_COMMAND}
            ${CMAKE_CORE_BUILD_FLAGS}
            -DENABLE_OPENMP=${AV_USE_OPENMP}
            -DENABLE_LCMS=ON
            -DENABLE_EXAMPLES=OFF
            ${ZLIB_CMAKE_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            -DINSTALL_CMAKE_MODULE_PATH:PATH=<INSTALL_DIR>/cmake
            -DCMAKE_INSTALL_LIBDIR=lib
            <SOURCE_DIR>
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS libraw_cmake ${ZLIB_TARGET} ${JPEG_TARGET}
    )

    set(LIBRAW_CMAKE_FLAGS
        -DLIBRAW_PATH=${CMAKE_INSTALL_PREFIX}
        -DPC_LIBRAW_INCLUDEDIR=${CMAKE_INSTALL_PREFIX}/include
        -DPC_LIBRAW_LIBDIR=${CMAKE_INSTALL_PREFIX}/lib
        -DPC_LIBRAW_R_LIBDIR=${CMAKE_INSTALL_PREFIX}/lib
    )
    av_register_dep(${LIBRAW_TARGET})
endif()
