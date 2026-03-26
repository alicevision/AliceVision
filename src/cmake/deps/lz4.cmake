# =============================================================================
# deps/lz4.cmake
# Dependencies: (none)
# Provides:     LZ4_TARGET, LZ4_CMAKE_FLAGS
#
# NOTE: lz4 places its CMakeLists.txt under build/cmake/ — cannot use
# av_add_cmake_dep which assumes CMakeLists.txt at SOURCE_DIR root.
# =============================================================================

if(AV_BUILD_FLANN)
    set(LZ4_TARGET lz4)

    ExternalProject_Add(${LZ4_TARGET}
        GIT_REPOSITORY  ${DEP_LZ4_GIT_REPO}
        GIT_TAG         ${DEP_LZ4_GIT_TAG}
        PREFIX          ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    0
        UPDATE_COMMAND  ""
        SOURCE_DIR      ${CMAKE_CURRENT_BINARY_DIR}/lz4
        BINARY_DIR      ${BUILD_DIR}/lz4_build
        INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND}
            ${CMAKE_CORE_BUILD_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            -DCMAKE_INSTALL_LIBDIR=lib
            <SOURCE_DIR>/build/cmake/
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
        INSTALL_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --target install
    )

    set(LZ4_CMAKE_FLAGS -Dlz4_DIR:PATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/lz4/)

    av_register_dep(${LZ4_TARGET})
endif()
