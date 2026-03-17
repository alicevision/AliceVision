# =============================================================================
# deps/usd.cmake
# Dependencies: (none — self-contained Python build)
# Provides:     USD_TARGET, USD_CMAKE_FLAGS
#
# NOTE: USD uses its own Python build script (build_usd.py), not CMake configure.
# =============================================================================

if(AV_BUILD_USD)
    set(USD_TARGET pxr)

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
