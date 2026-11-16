# Trained vocabulary tree used by ImageMatching (VocabularyTree methods).
#
# Included from the top-level CMakeLists.txt when ALICEVISION_INSTALL_VOCTREE is ON:
# defines the "voctree" target (part of ALL) that downloads the tree into the build
# directory, and installs it into share/aliceVision, where Meshroom looks for it.
#
# Also run in script mode by that target (cmake -P) to do the download itself:
#   -DVOCTREE_URL=<url> -DVOCTREE_SHA256=<hash> -DVOCTREE_OUTPUT=<file>

if(CMAKE_SCRIPT_MODE_FILE)
    # ── Script mode: download (skipped if the file is already there and valid) ──
    if(EXISTS "${VOCTREE_OUTPUT}")
        file(SHA256 "${VOCTREE_OUTPUT}" _existingHash)
        if(_existingHash STREQUAL VOCTREE_SHA256)
            return()
        endif()
        message(STATUS "Vocabulary tree hash mismatch, downloading it again: ${VOCTREE_OUTPUT}")
    endif()

    message(STATUS "Downloading the vocabulary tree: ${VOCTREE_URL}")
    # Download to a temporary name: an interrupted or corrupted download never leaves a file
    # that looks valid at the final path.
    set(_partial "${VOCTREE_OUTPUT}.part")
    # The hash is checked here rather than with EXPECTED_HASH, which raises its error
    # before the partial file can be removed.
    file(DOWNLOAD "${VOCTREE_URL}" "${_partial}"
         TLS_VERIFY ON
         STATUS _status)
    list(GET _status 0 _statusCode)
    if(NOT _statusCode EQUAL 0)
        file(REMOVE "${_partial}")
        list(GET _status 1 _statusMessage)
        message(FATAL_ERROR "Failed to download the vocabulary tree from ${VOCTREE_URL}: ${_statusMessage}")
    endif()
    file(SHA256 "${_partial}" _downloadedHash)
    if(NOT _downloadedHash STREQUAL VOCTREE_SHA256)
        file(REMOVE "${_partial}")
        message(FATAL_ERROR "Vocabulary tree hash mismatch for ${VOCTREE_URL}:\n"
                            "  expected: ${VOCTREE_SHA256}\n"
                            "  actual:   ${_downloadedHash}")
    endif()
    file(RENAME "${_partial}" "${VOCTREE_OUTPUT}")
    return()
endif()

# ── Configure mode ──

# Pinned commit of https://gitlab.com/alicevision/trainedVocabularyTreeData
set(ALICEVISION_VOCTREE_URL
    "https://gitlab.com/alicevision/trainedVocabularyTreeData/-/raw/f934e8252360c7d2d274c3902db9702e74a42560/vlfeat_K80L3.SIFT.tree")
set(ALICEVISION_VOCTREE_SHA256 "4ae79c2170fbb54a00195f866f2b4213ea2b4bc303f451b6b360709a1d194cb8")

set(_voctreeFile "${CMAKE_BINARY_DIR}/voctree/vlfeat_K80L3.SIFT.tree")

# Download at build time (not configure time), so that reconfiguring never downloads.
add_custom_target(voctree ALL
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/voctree"
    COMMAND ${CMAKE_COMMAND}
            -DVOCTREE_URL=${ALICEVISION_VOCTREE_URL}
            -DVOCTREE_SHA256=${ALICEVISION_VOCTREE_SHA256}
            -DVOCTREE_OUTPUT=${_voctreeFile}
            -P "${CMAKE_CURRENT_LIST_FILE}"
    COMMENT "Checking the vocabulary tree (downloads ~266 MB the first time)"
    VERBATIM
)

install(FILES "${_voctreeFile}"
    DESTINATION ${CMAKE_INSTALL_DATADIR}/aliceVision
)
