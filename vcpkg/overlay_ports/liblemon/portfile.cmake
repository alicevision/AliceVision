vcpkg_check_linkage(ONLY_STATIC_LIBRARY)

set(VERSION ed2c21cbd6ef)

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO alicevision/lemon
    REF 5493d5317605f56076e94b04caa6199d52957d74
    SHA512 ea46f74c70c30569b4c18fe54fe2cc157b8d0dc17b8052851d543fb5d2a24b28bf770ff7002519aaf6f9de02aa22926db4790b9615deac0f834d24c3c18df526
    HEAD_REF develop
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DLEMON_ENABLE_GLPK=OFF
        -DLEMON_ENABLE_ILOG=OFF
        -DLEMON_ENABLE_COIN=OFF
        -DLEMON_ENABLE_SOPLEX=OFF
)

vcpkg_cmake_install()

vcpkg_cmake_config_fixup(CONFIG_PATH share/lemon/cmake PACKAGE_NAME lemon)

vcpkg_fixup_pkgconfig()

file(GLOB EXE "${CURRENT_PACKAGES_DIR}/bin/*.exe")
file(COPY ${EXE} DESTINATION "${CURRENT_PACKAGES_DIR}/tools/liblemon/")
vcpkg_copy_tool_dependencies("${CURRENT_PACKAGES_DIR}/tools/liblemon")

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/share")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/bin" "${CURRENT_PACKAGES_DIR}/debug/bin")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/share/doc")

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
