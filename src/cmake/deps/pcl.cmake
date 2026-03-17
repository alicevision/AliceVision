# =============================================================================
# deps/pcl.cmake
# Dependencies: eigen, boost, png, flann, lz4, zlib, cuda (optional)
# Provides:     PCL_TARGET, PCL_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_PCL)
    set(PCL_TARGET pcl)

    av_add_cmake_dep(
        TARGET     ${PCL_TARGET}
        SOURCE_DIR pcl
        URL        ${DEP_PCL_URL}
        URL_HASH   ${DEP_PCL_HASH}
        EXTRA_CMAKE_FLAGS
            ${EIGEN_CMAKE_FLAGS}
            ${BOOST_CMAKE_FLAGS}
            ${PNG_CMAKE_FLAGS}
            ${CUDA_CMAKE_FLAGS}
            ${FLANN_CMAKE_FLAGS}
            ${LZ4_CMAKE_FLAGS}
            ${ZLIB_CMAKE_FLAGS}
            -DWITH_CUDA:BOOL=${AV_USE_CUDA}
            -DWITH_OPENGL:BOOL=OFF
            -DWITH_OPENMP:BOOL=ON
            -DWITH_LIBUSB:BOOL=OFF
            -DWITH_VTK:BOOL=OFF
            -DWITH_PCAP:BOOL=OFF
        DEPENDS
            ${FLANN_TARGET} ${LZ4_TARGET}
            ${EIGEN_TARGET} ${BOOST_TARGET}
            ${PNG_TARGET}   ${CUDA_TARGET}
            ${ZLIB_TARGET}
    )

    set(PCL_CMAKE_FLAGS -DPCL_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/pcl-1.12/)
endif()
