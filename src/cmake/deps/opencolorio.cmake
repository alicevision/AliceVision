# =============================================================================
# deps/opencolorio.cmake
# Dependencies: expat
# Provides:     OPENCOLORIO_TARGET, OPENCOLORIO_CMAKE_FLAGS
#
# NOTE: OCIO requires C++17 (not 20) — CXX_STANDARD is explicitly overridden.
# =============================================================================

if(AV_BUILD_OPENCOLORIO)
    set(OPENCOLORIO_TARGET opencolorio)

    ExternalProject_Add(${OPENCOLORIO_TARGET}
        GIT_REPOSITORY  ${DEP_OPENCOLORIO_GIT_REPO}
        GIT_TAG         ${DEP_OPENCOLORIO_GIT_TAG}
        PREFIX          ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    0
        SOURCE_DIR      ${CMAKE_CURRENT_BINARY_DIR}/OpenColorIO
        BINARY_DIR      ${BUILD_DIR}/OpenColorIO_build
        INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND}
            ${CMAKE_CORE_BUILD_FLAGS}
            -DCMAKE_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
            -DOCIO_BUILD_NUKE=OFF
            -DOCIO_BUILD_DOCS=OFF
            -DOCIO_BUILD_TESTS=OFF
            -DOCIO_BUILD_GPU_TESTS=OFF
            -DOCIO_BUILD_PYTHON=OFF
            -DOCIO_INSTALL_EXT_PACKAGES=MISSING
            -DCMAKE_CXX_STANDARD=17
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            -DCMAKE_INSTALL_LIBDIR=lib
            <SOURCE_DIR>
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS ${EXPAT_TARGET}
    )

    set(OPENCOLORIO_CMAKE_FLAGS -DOpenColorIO_DIR=${CMAKE_INSTALL_PREFIX})
    av_register_dep(${OPENCOLORIO_TARGET})
endif()
