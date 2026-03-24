vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO alicevision/popsift
    REF 77d1d28624d2849798fc107619507b50cd8cfe10
    SHA512 16a8ec7efb8ca8be8b558663a08ab431bba043ab6d6cb54b3ad1a2044e57e0c28cb3bfa50296a5817839f13eb7357317ff890e68b03becf5e6edbfb587a1b6cf
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
