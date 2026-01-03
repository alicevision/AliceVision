vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO alicevision/popsift
    REF abef1d48259b73248250379af4c4afaf8cff7f87
    SHA512 913f3694d21209801c7fa26aaf990ffe672d077450055610b92d2da6d5df5bc5198b4de817fe7fd25044aa886d1f8e38adfd955bb9159e64d9b55eb323fd830e
    HEAD_REF develop
)

vcpkg_find_cuda(OUT_CUDA_TOOLKIT_ROOT CUDA_TOOLKIT_ROOT)

vcpkg_check_features(OUT_FEATURE_OPTIONS FEATURE_OPTIONS
    FEATURES
        apps       PopSift_BUILD_EXAMPLES
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS ${FEATURE_OPTIONS}
        "-DCUDA_TOOLKIT_ROOT_DIR=${CUDA_TOOLKIT_ROOT}"
)

vcpkg_cmake_install()

vcpkg_cmake_config_fixup(CONFIG_PATH lib/cmake/PopSift)

vcpkg_copy_pdbs()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include" "${CURRENT_PACKAGES_DIR}/debug/share")

# copy the apps in tools directory
if ("apps" IN_LIST FEATURES)
    vcpkg_copy_tools(TOOL_NAMES popsift-demo AUTO_CLEAN)
endif()

file(INSTALL "${SOURCE_PATH}/COPYING.md" DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}" RENAME copyright)
