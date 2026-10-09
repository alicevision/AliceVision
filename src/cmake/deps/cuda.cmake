# =============================================================================
# deps/cuda.cmake
# Dependencies: (none)
# Provides:     CUDA_TARGET, CUDA_CMAKE_FLAGS, CUDA_ARCH_CMAKE_FLAGS, AV_CUDA_CC_LIST_ESC
# =============================================================================

if(AV_USE_CUDA AND AV_BUILD_CUDA)
    set(CUDA_TARGET cuda)

    set(_cuda_exe "cuda_${DEP_CUDA_VERSION}_${DEP_CUDA_DRIVER}_linux.run")

    ExternalProject_Add(${CUDA_TARGET}
        URL                 ${DEP_CUDA_URL}
        DOWNLOAD_NO_EXTRACT 1
        PREFIX              ${BUILD_DIR}
        BUILD_IN_SOURCE     0
        BUILD_ALWAYS        0
        UPDATE_COMMAND      ""
        SOURCE_DIR          ${CMAKE_CURRENT_BINARY_DIR}/cuda
        BINARY_DIR          ${BUILD_DIR}/cuda_build
        INSTALL_DIR         ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND   ""
        BUILD_COMMAND       ""
        INSTALL_COMMAND
            sh ${BUILD_DIR}/src/${_cuda_exe}
                --silent --no-opengl-libs --toolkit
                --toolkitpath=<INSTALL_DIR>
    )

    set(CUDA_CUDART_LIBRARY "")
    set(CUDA_CMAKE_FLAGS -DCUDA_TOOLKIT_ROOT_DIR=${CMAKE_INSTALL_PREFIX})

    av_register_dep(${CUDA_TARGET})

    unset(_cuda_exe)
elseif(AV_USE_CUDA)
    # Allow pointing to a pre-installed CUDA toolkit via cache variable
    option(CUDA_TOOLKIT_ROOT_DIR "Path to an existing CUDA toolkit installation" "")
    if(CUDA_TOOLKIT_ROOT_DIR)
        set(CUDA_CMAKE_FLAGS -DCUDA_TOOLKIT_ROOT_DIR=${CUDA_TOOLKIT_ROOT_DIR})
    endif()
endif()

if(AV_USE_CUDA)
    # CUDA CCs compiled into the dependencies: SASS for each CC (it also runs on the
    # later minors of the same major, e.g. sm_80 on 8.6/8.9) + PTX for the last one
    # so newer GPUs can JIT. Default: one CC per major supported by the toolkit
    # (same as CMake's "all-major"). Trim to the GPUs actually deployed to shrink further.
    if(AV_BUILD_CUDA)
        set(_cuda_version ${DEP_CUDA_VERSION})
    else()
        find_package(CUDAToolkit QUIET)
        set(_cuda_version ${CUDAToolkit_VERSION})
    endif()
    if(_cuda_version VERSION_GREATER_EQUAL 13.0)
        set(_cuda_default_cc "75;80;90;100;120")
    elseif(_cuda_version VERSION_GREATER_EQUAL 12.8)
        set(_cuda_default_cc "50;60;70;80;90;100;120")
    else()
        set(_cuda_default_cc "50;60;70;80;90")
    endif()
    set(AV_CUDA_CC_LIST "${_cuda_default_cc}" CACHE STRING "CUDA compute capabilities built into dependencies")

    list(TRANSFORM AV_CUDA_CC_LIST APPEND "-real" OUTPUT_VARIABLE _cuda_archs)
    list(GET AV_CUDA_CC_LIST -1 _cuda_last_cc)
    list(APPEND _cuda_archs "${_cuda_last_cc}-virtual")
    # $<SEMICOLON> keeps each list a single argument in the ExternalProject command line
    list(JOIN _cuda_archs "$<SEMICOLON>" _cuda_archs)
    list(JOIN AV_CUDA_CC_LIST "$<SEMICOLON>" AV_CUDA_CC_LIST_ESC)
    set(CUDA_ARCH_CMAKE_FLAGS -DCMAKE_CUDA_ARCHITECTURES=${_cuda_archs})
    unset(_cuda_version)
    unset(_cuda_default_cc)
    unset(_cuda_archs)
    unset(_cuda_last_cc)
endif()
