# =============================================================================
# deps/boost.cmake
# Dependencies: zlib
# Provides:     BOOST_TARGET, BOOST_CMAKE_FLAGS
#
# NOTE: Boost uses its own b2 build system, not CMake.
# =============================================================================

if(AV_BUILD_BOOST)
    set(BOOST_TARGET boost)

    if(WIN32)
        set(_boost_bootstrap_ext bat)
    else()
        set(_boost_bootstrap_ext sh)
    endif()

    ExternalProject_Add(${BOOST_TARGET}
        URL              ${DEP_BOOST_URL}
        URL_HASH         ${DEP_BOOST_HASH}
        DOWNLOAD_DIR     ${BUILD_DIR}/download/boost
        DOWNLOAD_NO_PROGRESS TRUE
        PREFIX           ${BUILD_DIR}
        BUILD_IN_SOURCE  0
        BUILD_ALWAYS     0
        UPDATE_COMMAND   ""
        SOURCE_DIR       ${CMAKE_CURRENT_BINARY_DIR}/boost
        BINARY_DIR       ${BUILD_DIR}/boost_build
        INSTALL_DIR      ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            cd <SOURCE_DIR> &&
            ./bootstrap.${_boost_bootstrap_ext}
                --prefix=<INSTALL_DIR>
                --with-libraries=atomic,container,date_time,exception,graph,iostreams,json,log,math,program_options,regex,serialization,system,test,thread,stacktrace,timer
        BUILD_COMMAND
            cd <SOURCE_DIR> &&
            ./b2 --prefix=<INSTALL_DIR>
                variant=${DEPS_CMAKE_BUILD_TYPE_LOWERCASE}
                cxxstd=20 link=shared threading=multi
                -j${AV_BUILD_DEPENDENCIES_PARALLEL}
        INSTALL_COMMAND
            cd <SOURCE_DIR> &&
            ./b2
                variant=${DEPS_CMAKE_BUILD_TYPE_LOWERCASE}
                cxxstd=20 link=shared threading=multi
                install
        DEPENDS ${ZLIB_TARGET}
    )

    set(BOOST_CMAKE_FLAGS -DBOOST_ROOT=${CMAKE_INSTALL_PREFIX})
    av_register_dep(${BOOST_TARGET})
    unset(_boost_bootstrap_ext)
endif()
