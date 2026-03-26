# =============================================================================
# deps/openimageio.cmake
# Dependencies: boost, openexr, tiff, png, jpeg, libraw, zlib, ffmpeg,
#               pybind11, expat, opencolorio
# Provides:     OPENIMAGEIO_TARGET, OPENIMAGEIO_CMAKE_FLAGS
# =============================================================================

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
            -DCMAKE_INSTALL_LIBDIR=lib
            <SOURCE_DIR>
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS
            ${BOOST_TARGET}
            ${OPENEXR_TARGET}   ${TIFF_TARGET}   ${PNG_TARGET}
            ${JPEG_TARGET}      ${LIBRAW_TARGET}  ${ZLIB_TARGET}
            ${FFMPEG_TARGET}    ${PYBIND11_TARGET}
            ${EXPAT_TARGET}     ${OPENCOLORIO_TARGET}
    )

    set(OPENIMAGEIO_CMAKE_FLAGS -DOpenImageIO_DIR=${CMAKE_INSTALL_PREFIX})
    av_register_dep(${OPENIMAGEIO_TARGET})
endif()
