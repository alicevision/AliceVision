# =============================================================================
# deps/openmp.cmake
# Dependencies: (none)
# Provides:     OPENMP_TARGET, OPENMP_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_OPENMP)
    set(OPENMP_TARGET openmp)

    # Default to shared OpenMP, static OpenMP can cause *many* problems,
    # including SIGSEGVs if not linked in correctly.
    # Let the user overwrite this if BUILD_SHARED_LIBS is false
    set(OPENMP_BUILD_SHARED_LIB ON)
    set(OPENMP_BUILD_STATIC_LIB OFF)
    if(NOT BUILD_SHARED_LIBS)
      set(OPENMP_BUILD_SHARED_LIB OFF)
      set(OPENMP_BUILD_STATIC_LIB ON)
    endif()

    av_add_cmake_dep(
        TARGET     ${OPENMP_TARGET}
        SOURCE_DIR llvm
        URL        ${DEP_OPENMP_URL}
        URL_HASH   ${DEP_OPENMP_HASH}
        EXTRA_CMAKE_FLAGS
          -DLIBOMP_ENABLE_SHARED=${OPENMP_BUILD_SHARED_LIB}
          -DLIBOMP_ENABLE_STATIC=${OPENMP_BUILD_STATIC_LIB}
          <SOURCE_DIR>/openmp
    )

    # Some sub-dependencies don't use the CMake OpenMP targets and instead
    # rely on setting the flags only. If using the Xcode Toolchain (Apple Clang
    # and Apple ld, we must explicitly pass these flags to ensure header paths
    # and linkage of libomp.
    if(APPLE)
      set(OPENMP_CMAKE_FLAGS
        -DCMAKE_C_FLAGS=-I${CMAKE_INSTALL_PREFIX}/include\ -Wl,-L${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}\ -Wl,-lomp
        -DCMAKE_CXX_FLAGS=-I${CMAKE_INSTALL_PREFIX}/include\ -Wl,-L${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}\ -Wl,-lomp
      )
    else()
      set(OPENMP_CMAKE_FLAGS)
    endif()
endif()
