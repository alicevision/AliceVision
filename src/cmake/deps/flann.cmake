# =============================================================================
# deps/flann.cmake
# Dependencies: lz4
# Provides:     FLANN_TARGET, FLANN_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_FLANN)
    set(FLANN_TARGET flann)

    ExternalProject_Add(${FLANN_TARGET}
        GIT_REPOSITORY  ${DEP_FLANN_GIT_REPO}
        GIT_TAG         ${DEP_FLANN_GIT_TAG}
        PREFIX          ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    0
        UPDATE_COMMAND  ""
        SOURCE_DIR      ${CMAKE_CURRENT_BINARY_DIR}/flann
        BINARY_DIR      ${BUILD_DIR}/flann_build
        INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND} -E env
            PKG_CONFIG_PATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/pkgconfig/
            ${CMAKE_COMMAND}
            ${CMAKE_CORE_BUILD_FLAGS}
            -DBUILD_C_BINDINGS:BOOL=OFF
            -DBUILD_EXAMPLES=OFF
            -DBUILD_TESTS:BOOL=OFF
            -DBUILD_DOC:BOOL=OFF
            -DBUILD_PYTHON_BINDINGS:BOOL=OFF
            -DBUILD_MATLAB_BINDINGS:BOOL=OFF
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
        DEPENDS ${LZ4_TARGET}
    )

set(FLANN_CMAKE_FLAGS -Dflann_DIR:PATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/flann/)
    av_register_dep(${FLANN_TARGET})
endif()
