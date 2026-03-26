# =============================================================================
# deps/swig.cmake
# Dependencies: (none)
# Provides:     SWIG_TARGET, SWIG_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_SWIG)
    set(SWIG_TARGET SWIG)

    #av_add_cmake_dep(
    #    TARGET         PCRE2
    #    SOURCE_DIR     pcre2
    #    GIT_REPOSITORY ${DEP_PCRE2_GIT_REPO}
    #    GIT_TAG        ${DEP_PCRE2_GIT_TAG}
    #)

    av_add_cmake_dep(
        TARGET         ${SWIG_TARGET}
        SOURCE_DIR     SWIG
        GIT_REPOSITORY ${DEP_SWIG_GIT_REPO}
        GIT_TAG        ${DEP_SWIG_GIT_TAG}
    #    DEPENDS        PCRE2
    )

    set(SWIG_CMAKE_FLAGS
        -DSWIG_DIR=${CMAKE_INSTALL_PREFIX}/share/swig/${DEP_SWIG_VERSION}
        -DSWIG_EXECUTABLE=${CMAKE_INSTALL_PREFIX}/bin-deps/swig
    )
endif()
