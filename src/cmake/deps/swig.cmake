# =============================================================================
# deps/swig.cmake
# Dependencies: (none)
# Provides:     SWIG_TARGET, SWIG_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_SWIG)
    set(SWIG_TARGET SWIG)

    # SWIG is a build-tool, so it always needs to match the host processor
    # if cross-compiling on Apple platforms, so force the correct value here
    if(APPLE)
      set(SWIG_HOST_PROCESSOR_ARCH ${CMAKE_HOST_SYSTEM_PROCESSOR})
    endif()

    av_add_cmake_dep(
        TARGET         ${SWIG_TARGET}
        SOURCE_DIR     SWIG
        GIT_REPOSITORY ${DEP_SWIG_GIT_REPO}
        GIT_TAG        ${DEP_SWIG_GIT_TAG}
        EXTRA_CMAKE_FLAGS
          -DWITH_PCRE=OFF
          -DCMAKE_OSX_ARCHITECTURES=${SWIG_HOST_PROCESSOR_ARCH}
    )

    if(APPLE)
      set(SWIG_CMAKE_FLAGS
          -DSWIG_DIR=${CMAKE_INSTALL_PREFIX}/share/swig/${DEP_SWIG_VERSION}
          -DSWIG_EXECUTABLE=${CMAKE_INSTALL_PREFIX}/bin/swig
      )
    else()
      set(SWIG_CMAKE_FLAGS
          -DSWIG_DIR=${CMAKE_INSTALL_PREFIX}/share/swig/${DEP_SWIG_VERSION}
          -DSWIG_EXECUTABLE=${CMAKE_INSTALL_PREFIX}/bin-deps/swig
      )
    endif()
endif()
