set(VCPKG_BUILD_TYPE "release")
set(VCPKG_POLICY_EMPTY_INCLUDE_FOLDER enabled)

vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO swig/swig
    REF "v${VERSION}"
    SHA512 0a9bdcc4a28f1374f9e564fdb372f684bd277bf7b590491218c3538c230bcacd8de32365717a37cf5d7d02165dec9f3955e1bb1207181885fb5b7fbc7476ff04
    HEAD_REF master
)

vcpkg_find_acquire_program(BISON)
vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        "-DBISON_EXECUTABLE=${BISON}"
)

vcpkg_cmake_install()
vcpkg_fixup_pkgconfig()

vcpkg_copy_tools(
    TOOL_NAMES swig
    DESTINATION "${CURRENT_PACKAGES_DIR}/tools/swig"
)

file(COPY "${CURRENT_PACKAGES_DIR}/bin/" DESTINATION "${CURRENT_PACKAGES_DIR}/tools/swig")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/bin")

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
