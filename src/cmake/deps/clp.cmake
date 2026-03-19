# =============================================================================
# deps/clp.cmake
# Dependencies: coinutils, osi
# Provides:     CLP_TARGET, CLP_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_CLP)
    set(CLP_TARGET clp)

    av_add_cmake_dep(
        TARGET         ${CLP_TARGET}
        SOURCE_DIR     clp
        GIT_REPOSITORY ${DEP_CLP_GIT_REPO}
        GIT_TAG        ${DEP_CLP_GIT_TAG}
        DEPENDS        ${COINUTILS_TARGET} ${OSI_TARGET}
        EXTRA_CMAKE_FLAGS
            -DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=TRUE
    )

    set(CLP_CMAKE_FLAGS -DClp_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/clp)
endif()
