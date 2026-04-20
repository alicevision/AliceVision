# =============================================================================
# deps/xercesc.cmake
# Dependencies: (none)
# Provides:     XERCESC_TARGET, XERCESC_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_XERCESC)
    set(XERCESC_TARGET xercesc)

    av_add_cmake_dep(
        TARGET     ${XERCESC_TARGET}
        SOURCE_DIR xercesc
        URL        ${DEP_XERCESC_URL}
        URL_HASH   ${DEP_XERCESC_HASH}
    )

    set(XERCESC_CMAKE_FLAGS
      -DXercesC_DIR:PATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/XercesC
    )
endif()
