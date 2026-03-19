# =============================================================================
# deps/pybind11.cmake
# Dependencies: (none)
# Provides:     PYBIND11_TARGET
# =============================================================================

if(AV_BUILD_PYBIND11)
    set(PYBIND11_TARGET pybind11)

    av_add_cmake_dep(
        TARGET         ${PYBIND11_TARGET}
        SOURCE_DIR     pybind11
        GIT_REPOSITORY ${DEP_PYBIND11_GIT_REPO}
        GIT_TAG        ${DEP_PYBIND11_GIT_TAG}
        EXTRA_CMAKE_FLAGS
            -DPython_EXECUTABLE=${Python_EXECUTABLE}
            -DCMAKE_INSTALL_DATAROOTDIR=lib
    )
endif()
