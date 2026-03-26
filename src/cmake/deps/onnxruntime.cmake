# =============================================================================
# deps/onnxruntime.cmake
# Dependencies: (none — pre-built binary, download + copy only)
# Provides:     ONNXRUNTIME_TARGET
#
# NOTE: Platform-specific archive — resolved at configure time from host OS/arch.
# =============================================================================

if(AV_BUILD_ONNXRUNTIME)
    if(APPLE)
        set(_onnx_prefix "onnxruntime-osx-${AV_ONNX_APPLE_ARCH}")
        if(AV_ONNX_APPLE_ARCH STREQUAL "arm64")
            set(_onnx_hash ${DEP_ONNXRUNTIME_OSX_ARM64_HASH})
        elseif(AV_ONNX_APPLE_ARCH STREQUAL "x86_64")
            set(_onnx_hash ${DEP_ONNXRUNTIME_OSX_X86_64_HASH})
        else()
            message(FATAL_ERROR
                "ONNX Runtime: unsupported Apple arch '${AV_ONNX_APPLE_ARCH}'. "
                "Expected arm64 or x86_64.")
        endif()
    else()
        string(FIND "${CMAKE_HOST_SYSTEM_PROCESSOR}" "aarch64" _onnx_aarch64_pos)
        if(NOT _onnx_aarch64_pos EQUAL -1)
            set(_onnx_prefix "onnxruntime-linux-aarch64")
            set(_onnx_hash ${DEP_ONNXRUNTIME_LINUX_AARCH64_HASH})
        else()
            set(_onnx_prefix "onnxruntime-linux-x64")
            set(_onnx_hash ${DEP_ONNXRUNTIME_LINUX_X64_HASH})
        endif()
    endif()

    set(ONNXRUNTIME_TARGET onnxruntime)

    ExternalProject_Add(${ONNXRUNTIME_TARGET}
        URL              "https://github.com/microsoft/onnxruntime/releases/download/v${DEP_ONNXRUNTIME_VERSION}/${_onnx_prefix}-${DEP_ONNXRUNTIME_VERSION}.tgz"
        URL_HASH         ${_onnx_hash}
        DOWNLOAD_DIR     ${BUILD_DIR}/download/onnxruntime
        DOWNLOAD_NO_PROGRESS TRUE
        PREFIX           ${BUILD_DIR}
        BUILD_IN_SOURCE  0
        BUILD_ALWAYS     0
        UPDATE_COMMAND   ""
        SOURCE_DIR       ${CMAKE_CURRENT_BINARY_DIR}/onnxruntime
        INSTALL_DIR      ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND ""
        BUILD_COMMAND     ""
        INSTALL_COMMAND
            sh -c
            "mkdir -p <INSTALL_DIR>/include <INSTALL_DIR>/lib \
             && cp -r <SOURCE_DIR>/lib/*     <INSTALL_DIR>/lib \
             && cp -r <SOURCE_DIR>/include/* <INSTALL_DIR>/include"
    )

    av_register_dep(${ONNXRUNTIME_TARGET})
    unset(_onnx_prefix)
    unset(_onnx_hash)
    unset(_onnx_aarch64_pos)
endif()
