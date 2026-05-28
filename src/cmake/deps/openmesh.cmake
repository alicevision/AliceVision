# =============================================================================
# deps/openmesh.cmake
# Dependencies: (none)
# Provides:     OPENMESH_TARGET, OPENMESH_CMAKE_FLAGS
#
# NOTE: OpenMesh always builds in Release regardless of the parent build type.
# =============================================================================

if(AV_BUILD_OPENMESH)
    set(OPENMESH_TARGET OpenMesh)

    if(UNIX)
        set(_fix_pkgfile ${CMAKE_COMMAND} -E rename <INSTALL_DIR>/libdata/pkgconfig/openmesh.pc <INSTALL_DIR>/lib/pkgconfig/openmesh.pc && ${CMAKE_COMMAND} -E rm -r <INSTALL_DIR>/libdata)
    else()
        set(_fix_pkgfile "")
    endif()

    ExternalProject_Add(${OPENMESH_TARGET}
        URL              ${DEP_OPENMESH_URL}
        URL_HASH         ${DEP_OPENMESH_HASH}
        DOWNLOAD_DIR     ${BUILD_DIR}/download/OpenMesh
        DOWNLOAD_NO_PROGRESS TRUE
        PREFIX           ${BUILD_DIR}
        BUILD_IN_SOURCE  0
        BUILD_ALWAYS     0
        UPDATE_COMMAND   ""
        SOURCE_DIR       ${CMAKE_CURRENT_BINARY_DIR}/OpenMesh
        BINARY_DIR       ${BUILD_DIR}/OpenMesh_build
        INSTALL_DIR      ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND}
            ${CMAKE_CORE_BUILD_FLAGS}
            -DBUILD_APPS=OFF
            -DOPENMESH_DOCS=OFF
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config Release
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
        INSTALL_COMMAND
            ${CMAKE_COMMAND} --install <BINARY_DIR>
                COMMAND
            ${_fix_pkgfile}
    )

    set(OPENMESH_CMAKE_FLAGS
        -DOPENMESH_DIR:PATH=${CMAKE_INSTALL_PREFIX}/share/OpenMesh/cmake
    )
    av_register_dep(${OPENMESH_TARGET})

    unset(_fix_pkgfile)
endif()
