# =============================================================================
# deps/nanoflann.cmake
# Dependencies: (none)
# Provides:     NANOFLANN_TARGET, NANOFLANN_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_NANOFLANN)
    set(NANOFLANN_TARGET nanoflann)

    ExternalProject_Add(${NANOFLANN_TARGET}
        GIT_REPOSITORY  ${DEP_NANOFLANN_GIT_REPO}
        GIT_TAG         ${DEP_NANOFLANN_GIT_TAG}
        PREFIX          ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    0
        UPDATE_COMMAND  ""
        SOURCE_DIR      ${CMAKE_CURRENT_BINARY_DIR}/nanoflann
        BINARY_DIR      ${BUILD_DIR}/nanoflann_build
        INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND} -E env
                PKG_CONFIG_PATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/pkgconfig/
            ${CMAKE_COMMAND}
            ${CMAKE_CORE_BUILD_FLAGS}
            -DNANOFLANN_BUILD_EXAMPLES=OFF
            -DNANOFLANN_BUILD_TESTS=OFF
            -DCMAKE_INSTALL_LIBDIR=lib
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
        INSTALL_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --target install
    )

    set(NANOFLANN_CMAKE_FLAGS
        -Dnanoflann_DIR:PATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/nanoflann/
    )
    av_register_dep(${NANOFLANN_TARGET})
endif()
