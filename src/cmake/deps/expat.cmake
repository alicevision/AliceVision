# =============================================================================
# deps/expat.cmake
# Dependencies: (none)
# Provides:     EXPAT_TARGET
#
# NOTE: libexpat places its CMakeLists.txt inside the "expat/" subdirectory,
# one level below the repo root — hence the explicit CONFIGURE_COMMAND.
# =============================================================================

if(AV_BUILD_EXPAT)
    set(EXPAT_TARGET expat)

    ExternalProject_Add(${EXPAT_TARGET}
        GIT_REPOSITORY  ${DEP_EXPAT_GIT_REPO}
        GIT_TAG         ${DEP_EXPAT_GIT_TAG}
        PREFIX          ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    0
        SOURCE_DIR      ${CMAKE_CURRENT_BINARY_DIR}/expat
        BINARY_DIR      ${BUILD_DIR}/libexpat_build
        INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND}
            ${CMAKE_CORE_BUILD_FLAGS}
            -DEXPAT_BUILD_DOCS:BOOL=OFF
            -DEXPAT_BUILD_EXAMPLES:BOOL=OFF
            -DEXPAT_BUILD_TOOLS:BOOL=OFF
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            -DCMAKE_INSTALL_LIBDIR=lib
            <SOURCE_DIR>/expat
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
    )

    av_register_dep(${EXPAT_TARGET})
endif()
