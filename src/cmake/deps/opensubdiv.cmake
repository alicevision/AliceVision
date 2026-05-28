# =============================================================================
# deps/opensubdiv.cmake
# Dependencies: (none — self-contained Python build)
# Provides:    None
# =============================================================================

if(AV_BUILD_OPENSUBDIV)
    set(OPENSUBDIV_TARGET opensubdiv)

    av_add_cmake_dep(
        TARGET         ${OPENSUBDIV_TARGET}
        SOURCE_DIR     opensubdiv
        GIT_REPOSITORY ${DEP_OPENSUBDIV_GIT_REPO}
        GIT_TAG        ${DEP_OPENSUBDIV_GIT_TAG}
        EXTRA_CMAKE_FLAGS
            # Disable all examples/tests/docs
            -DNO_EXAMPLES=ON
            -DNO_TUTORIALS=ON
            -DNO_REGRESSION=ON
            -DNO_DOC=ON
            -DNO_TESTS=ON
            -DNO_GLTESTS=ON
            
            # Disable ALL GPU/Graphics backends
            -DNO_OPENGL=ON
            -DNO_DX=ON
            -DNO_CUDA=ON
            -DNO_OPENCL=ON
            -DNO_METAL=ON
            -DNO_CLEW=ON
            -DNO_CUEW=ON

            # Disable PTex (requires OpenGL)
            -DNO_PTEX=ON

            # Enable CPU backends
            -DNO_OMP=OFF
            -DNO_TBB=OFF

            ${OPENMP_CMAKE_FLAGS}
        DEPENDS ${OPENMP_TARGET}
    )

endif()
