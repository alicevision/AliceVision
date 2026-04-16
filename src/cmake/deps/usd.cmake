# =============================================================================
# deps/usd.cmake
# Dependencies: (none — self-contained Python build)
# Provides:     USD_TARGET, USD_CMAKE_FLAGS
#
# NOTE: USD uses its own Python build script (build_usd.py), not CMake configure.
# =============================================================================

if(AV_BUILD_USD)
    set(USD_TARGET pxr)

    av_add_cmake_dep(
        TARGET         ${USD_TARGET}
        SOURCE_DIR     usd
        GIT_REPOSITORY ${DEP_USD_GIT_REPO}
        GIT_TAG        ${DEP_USD_GIT_TAG}
        EXTRA_CMAKE_FLAGS     
            -DPXR_BUILD_TESTS=OFF
            -DPXR_BUILD_EXAMPLES=OFF
            -DPXR_BUILD_TUTORIALS=OFF
            -DPXR_BUILD_USD_TOOLS=OFF
            -DPXR_BUILD_IMAGING=ON
            -DPXR_BUILD_USD_IMAGING=ON
            -DPXR_ENABLE_PYTHON_SUPPORT=OFF   
            -DPXR_ENABLE_GL_SUPPORT=OFF
            -DPXR_ENABLE_METAL_SUPPORT=OFF
            -DPXR_ENABLE_VULKAN_SUPPORT=OFF
            -DPXR_BUILD_GPU_SUPPORT=OFF
        DEPENDS
            ${TBB_TARGET}
            ${OPENSUBDIV_TARGET}
    )

    set(USD_CMAKE_FLAGS -Dpxr_DIR=${CMAKE_INSTALL_PREFIX})
endif()
