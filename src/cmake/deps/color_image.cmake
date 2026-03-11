# =============================================================================
# deps/color_image.cmake — Color management, image I/O and computer vision
#
# Dependencies: zlib, tiff, png, jpeg, libraw, openexr, boost, tbb, ffmpeg
# Provides:     OPENCV_CMAKE_FLAGS, OPENCOLORIO_CMAKE_FLAGS,
#               OPENIMAGEIO_CMAKE_FLAGS
# =============================================================================

# ── ONNX Runtime (pre-built binary distribution) ──────────────────────────────

if(AV_BUILD_ONNXRUNTIME)
    # Resolve the platform-specific filename prefix and hash
    if(APPLE)
        set(_onnx_prefix "onnxruntime-osx-${AV_ONNX_APPLE_ARCH}")
        if(AV_ONNX_APPLE_ARCH STREQUAL "arm64")
            set(_onnx_hash ${DEP_ONNXRUNTIME_OSX_ARM64_HASH})
        elseif(AV_ONNX_APPLE_ARCH STREQUAL "x86_64")
            set(_onnx_hash ${DEP_ONNXRUNTIME_OSX_X86_64_HASH})
        else()
            message(FATAL_ERROR "ONNX Runtime: unsupported Apple arch '${AV_ONNX_APPLE_ARCH}'. Expected arm64 or x86_64.")
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

# ── OpenColorIO ───────────────────────────────────────────────────────────────

if(AV_BUILD_OPENCOLORIO)
    set(OPENCOLORIO_TARGET opencolorio)

    # OCIO requires C++17 (not 20), hence explicit override of CXX_STANDARD.
    ExternalProject_Add(${OPENCOLORIO_TARGET}
        GIT_REPOSITORY  ${DEP_OPENCOLORIO_GIT_REPO}
        GIT_TAG         ${DEP_OPENCOLORIO_GIT_TAG}
        PREFIX          ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    0
        UPDATE_COMMAND  ""
        SOURCE_DIR      ${CMAKE_CURRENT_BINARY_DIR}/OpenColorIO
        BINARY_DIR      ${BUILD_DIR}/OpenColorIO_build
        INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND}
            ${CMAKE_CORE_BUILD_FLAGS}
            -DCMAKE_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
            -DOCIO_BUILD_NUKE=OFF
            -DOCIO_BUILD_DOCS=OFF
            -DOCIO_BUILD_TESTS=OFF
            -DOCIO_BUILD_GPU_TESTS=OFF
            -DOCIO_BUILD_PYTHON=OFF
            -DOCIO_INSTALL_EXT_PACKAGES=MISSING
            -DCMAKE_CXX_STANDARD=17
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
    )

    set(OPENCOLORIO_CMAKE_FLAGS -DOpenColorIO_DIR=${CMAKE_INSTALL_PREFIX})
    av_register_dep(${OPENCOLORIO_TARGET})
endif()

# ── OpenImageIO ───────────────────────────────────────────────────────────────

if(AV_BUILD_OPENIMAGEIO)
    set(OPENIMAGEIO_TARGET openimageio)

    ExternalProject_Add(${OPENIMAGEIO_TARGET}
        URL              ${DEP_OPENIMAGEIO_URL}
        URL_HASH         ${DEP_OPENIMAGEIO_HASH}
        DOWNLOAD_DIR     ${BUILD_DIR}/download/oiio
        DOWNLOAD_NO_PROGRESS TRUE
        PREFIX           ${BUILD_DIR}
        BUILD_IN_SOURCE  0
        BUILD_ALWAYS     0
        UPDATE_COMMAND   ""
        SOURCE_DIR       ${CMAKE_CURRENT_BINARY_DIR}/openimageio
        BINARY_DIR       ${BUILD_DIR}/openimageio_build
        INSTALL_DIR      ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND}
            ${CMAKE_CORE_BUILD_FLAGS}
            -DCMAKE_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}
            -DBOOST_ROOT=${CMAKE_INSTALL_PREFIX}
            -DOIIO_BUILD_TESTS:BOOL=OFF
            -DOIIO_BUILD_TOOLS:BOOL=OFF
            -DILMBASE_HOME=${CMAKE_INSTALL_PREFIX}
            -DOPENEXR_HOME=${CMAKE_INSTALL_PREFIX}
            ${TIFF_CMAKE_FLAGS}
            ${ZLIB_CMAKE_FLAGS}
            ${PNG_CMAKE_FLAGS}
            ${JPEG_CMAKE_FLAGS}
            ${LIBRAW_CMAKE_FLAGS}
            ${OPENEXR_CMAKE_FLAGS}
            ${OPENCOLORIO_CMAKE_FLAGS}
            -DSTOP_ON_WARNING=OFF
            -DUSE_FFMPEG=${AV_BUILD_FFMPEG}
            -DUSE_TURBOJPEG=${AV_BUILD_JPEG}
            -DUSE_LIBRAW=${AV_BUILD_LIBRAW}
            -DUSE_OPENEXR=${AV_BUILD_OPENEXR}
            -DUSE_TIFF=${AV_BUILD_TIFF}
            -DUSE_PNG=${AV_BUILD_PNG}
            -DPython3_EXECUTABLE=${Python_EXECUTABLE}
            -DUSE_PYTHON=ON
            -DUSE_OPENCV=OFF
            -DUSE_OPENGL=OFF
            -DUSE_NUKE=OFF
            -DUSE_PTEX=OFF
            -DUSE_FREETYPE=OFF
            -DUSE_JXL=OFF
            -DBUILD_DOCS=OFF
            -DBUILD_TESTING=OFF
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS
            ${BOOST_TARGET}
            ${OPENEXR_TARGET}   ${TIFF_TARGET}  ${PNG_TARGET}
            ${JPEG_TARGET}      ${LIBRAW_TARGET} ${ZLIB_TARGET}
            ${FFMPEG_TARGET}    ${PYBIND11_TARGET}
            ${EXPAT_TARGET}     ${OPENCOLORIO_TARGET}
    )

    set(OPENIMAGEIO_CMAKE_FLAGS -DOpenImageIO_DIR=${CMAKE_INSTALL_PREFIX})
    av_register_dep(${OPENIMAGEIO_TARGET})
endif()

# ── OpenCV ────────────────────────────────────────────────────────────────────

if(AV_BUILD_OPENCV)
    set(OPENCV_TARGET opencv)

    # Contrib modules: download only, no configure/build/install step
    ExternalProject_Add(opencv_contrib
        URL              ${DEP_OPENCV_CONTRIB_URL}
        URL_HASH         ${DEP_OPENCV_CONTRIB_HASH}
        DOWNLOAD_DIR     ${BUILD_DIR}/download/opencv_contrib
        DOWNLOAD_NO_PROGRESS TRUE
        SOURCE_DIR       ${CMAKE_CURRENT_BINARY_DIR}/opencv_contrib
        BUILD_ALWAYS     0
        UPDATE_COMMAND   ""
        CONFIGURE_COMMAND ""
        BUILD_COMMAND    ""
        INSTALL_COMMAND  ""
    )

    ExternalProject_Add(${OPENCV_TARGET}
        URL              ${DEP_OPENCV_URL}
        URL_HASH         ${DEP_OPENCV_HASH}
        DOWNLOAD_DIR     ${BUILD_DIR}/download/opencv
        DOWNLOAD_NO_PROGRESS TRUE
        UPDATE_COMMAND   ""
        BUILD_IN_SOURCE  0
        BUILD_ALWAYS     0
        SOURCE_DIR       ${CMAKE_CURRENT_BINARY_DIR}/opencv
        BINARY_DIR       ${BUILD_DIR}/opencv_build
        INSTALL_DIR      ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND}
            ${CMAKE_CORE_BUILD_FLAGS}
            -DOPENCV_EXTRA_MODULES_PATH=${CMAKE_CURRENT_BINARY_DIR}/opencv_contrib/modules
            ${ZLIB_CMAKE_FLAGS}
            ${TBB_CMAKE_FLAGS}
            ${FFMPEG_CMAKE_FLAGS}
            ${TIFF_CMAKE_FLAGS}
            ${PNG_CMAKE_FLAGS}
            ${JPEG_CMAKE_FLAGS}
            ${LIBRAW_CMAKE_FLAGS}
            -DWITH_TBB=ON
            -DWITH_FFMPEG=${AV_BUILD_FFMPEG}
            -DBUILD_opencv_python2=OFF
            -DBUILD_opencv_python3=ON
            -DWITH_GTK_2_X=OFF
            -DWITH_V4L=OFF
            -DINSTALL_C_EXAMPLES=OFF
            -DINSTALL_PYTHON_EXAMPLES=OFF
            -DBUILD_EXAMPLES=OFF
            -DWITH_QT=OFF
            -DWITH_OPENGL=OFF
            -DWITH_VTK=OFF
            # OpenEXR disabled: IlmBase includes require "OpenEXR/" prefix not handled here
            -DWITH_OPENEXR=OFF
            -DENABLE_PRECOMPILED_HEADERS=OFF
            -DBUILD_SHARED_LIBS=ON
            -DWITH_CUDA=OFF
            -DWITH_OPENCL=OFF
            -DBUILD_TESTS=OFF
            -DBUILD_LIST=core,improc,photo,objdetect,video,imgcodecs,videoio,features2d,xfeatures2d,version,mcc,optflow
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS
            opencv_contrib
            ${TBB_TARGET}   ${ZLIB_TARGET}  ${OPENEXR_TARGET}
            ${TIFF_TARGET}  ${PNG_TARGET}   ${JPEG_TARGET}
            ${LIBRAW_TARGET} ${FFMPEG_TARGET}
    )

    set(OPENCV_CMAKE_FLAGS
        -DOpenCV_DIR=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/opencv4
        -DOPENCV_DIR=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/opencv4
    )
    av_register_dep(${OPENCV_TARGET})
endif()
