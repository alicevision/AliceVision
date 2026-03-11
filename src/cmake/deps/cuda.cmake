# =============================================================================
# deps/cuda.cmake — CUDA toolkit (optional embedded install)
#
# Dependencies: (none)
# Provides:     CUDA_CMAKE_FLAGS
# =============================================================================

if(AV_USE_CUDA AND AV_BUILD_CUDA)
    set(CUDA_TARGET cuda)

    set(_cuda_exe "cuda_${AV_VERSION_CUDA}_${AV_CUDA_DRIVER}_linux.run")

    ExternalProject_Add(${CUDA_TARGET}
        URL                 ${AV_URL_CUDA}
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
else()
    # Allow pointing to a pre-installed CUDA toolkit via cache variable
    option(CUDA_TOOLKIT_ROOT_DIR "Path to an existing CUDA toolkit installation" "")
    if(CUDA_TOOLKIT_ROOT_DIR)
        set(CUDA_CMAKE_FLAGS -DCUDA_TOOLKIT_ROOT_DIR=${CUDA_TOOLKIT_ROOT_DIR})
    endif()
endif()
