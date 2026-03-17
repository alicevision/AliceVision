# =============================================================================
# deps/alembic.cmake
# Dependencies: boost, openexr, zlib
# Provides:     ALEMBIC_TARGET, ALEMBIC_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_ALEMBIC)
    set(ALEMBIC_TARGET alembic)

    av_add_cmake_dep(
        TARGET     ${ALEMBIC_TARGET}
        SOURCE_DIR alembic
        URL        ${DEP_ALEMBIC_URL}
        URL_HASH   ${DEP_ALEMBIC_HASH}
        EXTRA_CMAKE_FLAGS
            ${ZLIB_CMAKE_FLAGS}
            ${ILMBASE_CMAKE_FLAGS}
            -DUSE_TESTS=OFF
        DEPENDS ${BOOST_TARGET} ${OPENEXR_TARGET} ${ZLIB_TARGET}
    )

    set(ALEMBIC_CMAKE_FLAGS
        -DAlembic_DIR:PATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/cmake/Alembic
    )
endif()
