# =============================================================================
# deps/opencv.cmake
# Dependencies: tbb, zlib, openexr, tiff, png, jpeg, libraw, ffmpeg
# Provides:     OPENCV_TARGET, OPENCV_CMAKE_FLAGS
# =============================================================================

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
            -DCMAKE_INSTALL_LIBDIR=lib
            <SOURCE_DIR>
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS
            opencv_contrib
            ${TBB_TARGET}    ${ZLIB_TARGET}   ${OPENEXR_TARGET}
            ${TIFF_TARGET}   ${PNG_TARGET}    ${JPEG_TARGET}
            ${LIBRAW_TARGET} ${FFMPEG_TARGET}
    )

    set(OPENCV_CMAKE_FLAGS
        -DOpenCV_DIR=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/opencv4
        -DOPENCV_DIR=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/opencv4
    )
    av_register_dep(${OPENCV_TARGET})
endif()
