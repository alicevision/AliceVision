# =============================================================================
# macros.cmake
#
# Shared CMake helper macros for the AliceVision dependency build system.
# Must be included after versions.cmake and before any deps/*.cmake file.
# =============================================================================

# -----------------------------------------------------------------------------
# av_register_dep(TARGET)
#
# Appends TARGET to the global AV_DEPS list, which is later passed as the
# DEPENDS list of the top-level aliceVision ExternalProject.
# -----------------------------------------------------------------------------
macro(av_register_dep TARGET)
    list(APPEND AV_DEPS ${TARGET})
endmacro()

# -----------------------------------------------------------------------------
# av_add_cmake_dep(
#     TARGET             <target-name>
#     SOURCE_DIR         <relative-subdir-name>   # under CMAKE_CURRENT_BINARY_DIR
#     [URL               <url>]
#     [URL_HASH          <ALGO=hash>]
#     [GIT_REPOSITORY    <repo>]
#     [GIT_TAG           <tag-or-commit>]
#     [EXTRA_CMAKE_FLAGS <flag> ...]   # appended after CMAKE_CORE_BUILD_FLAGS
#     [DEPENDS           <dep> ...]
#     [DOWNLOAD_DIR      <absolute-path>]
#     [NO_REGISTER]                   # skip av_register_dep (rare)
# )
#
# Registers a standard CMake-based ExternalProject with sensible defaults:
#   - PREFIX / BINARY_DIR derived from BUILD_DIR and TARGET name
#   - INSTALL_DIR = CMAKE_INSTALL_PREFIX
#   - BUILD_COMMAND = cmake --build (generator-agnostic: works with Ninja, Make, etc.)
#   - UPDATE_COMMAND = ""  (reproducible builds)
# -----------------------------------------------------------------------------
macro(av_add_cmake_dep)
    cmake_parse_arguments(
        _D                          # prefix
        "NO_REGISTER"               # options
        "TARGET;SOURCE_DIR;URL;URL_HASH;GIT_REPOSITORY;GIT_TAG;DOWNLOAD_DIR"
        "EXTRA_CMAKE_FLAGS;DEPENDS"
        ${ARGN}
    )

    if(NOT _D_DOWNLOAD_DIR)
        set(_D_DOWNLOAD_DIR "${BUILD_DIR}/download/${_D_TARGET}")
    endif()

    # Build source arguments depending on fetch method
    set(_D_SRC_ARGS "")
    if(_D_URL)
        list(APPEND _D_SRC_ARGS
            URL              ${_D_URL}
            DOWNLOAD_DIR     ${_D_DOWNLOAD_DIR}
            DOWNLOAD_NO_PROGRESS TRUE
        )
        if(_D_URL_HASH)
            list(APPEND _D_SRC_ARGS URL_HASH ${_D_URL_HASH})
        endif()
    elseif(_D_GIT_REPOSITORY)
        list(APPEND _D_SRC_ARGS GIT_REPOSITORY ${_D_GIT_REPOSITORY})
        if(_D_GIT_TAG)
            list(APPEND _D_SRC_ARGS GIT_TAG ${_D_GIT_TAG})
        endif()
    endif()

    ExternalProject_Add(${_D_TARGET}
        ${_D_SRC_ARGS}
        PREFIX          ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    0
        UPDATE_COMMAND  ""
        SOURCE_DIR      ${CMAKE_CURRENT_BINARY_DIR}/${_D_SOURCE_DIR}
        BINARY_DIR      ${BUILD_DIR}/${_D_TARGET}_build
        INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND}
            ${CMAKE_CORE_BUILD_FLAGS}
            ${_D_EXTRA_CMAKE_FLAGS}
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS         ${_D_DEPENDS}
    )

    if(NOT _D_NO_REGISTER)
        av_register_dep(${_D_TARGET})
    endif()

    # Clean up local variables to avoid leaking between macro calls
    unset(_D_SRC_ARGS)
    unset(_D_DOWNLOAD_DIR)
endmacro()
