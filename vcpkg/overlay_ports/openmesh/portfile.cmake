# Note: upstream GitLab instance at https://graphics.rwth-aachen.de:9000 often goes down, use the GitHub mirror
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO meshroomHubWarehouse/OpenMesh
    REF OpenMesh-${VERSION}
    SHA512 90550eeb458f252c6251a0e31c708d5469704093d9297893830808daa6dec13c3241ad9338a2820f9591ed6f238bd6cf6b8f411eff0fafe10aa4f1c25b5ed793
    HEAD_REF master
    PATCHES
        fix-library-install-path.patch
        fix-pkgconfig.patch
        support-arm64-win.patch
)

# cmake-library is a git submodule, not included in the GitHub archive
vcpkg_from_github(
    OUT_SOURCE_PATH CMAKE_LIBRARY_SOURCE_PATH
    REPO meshroomHubWarehouse/cmake-library
    REF 9e180cdb4df1d8370ccd9aeee1499709441dbdd1
    SHA512 796ce9ebf2462c26561964f7f308c7e37dfab9def61027270985a6ebbbd8e6e480681a714e32781bb041a4734711a7efa99393dd5439724f3ad71a0a80967493
    HEAD_REF master
    PATCHES
        fix-cmake-library-install-path.patch
)
file(REMOVE_RECURSE "${SOURCE_PATH}/cmake-library")
file(COPY "${CMAKE_LIBRARY_SOURCE_PATH}/" DESTINATION "${SOURCE_PATH}/cmake-library")

if(VCPKG_LIBRARY_LINKAGE STREQUAL "dynamic")
  set(OPENMESH_BUILD_SHARED ON)
else()
  set(OPENMESH_BUILD_SHARED OFF)
endif()

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        -DBUILD_APPS=OFF
        -DVCI_COMMON_DO_NOT_COPY_POST_BUILD=ON
        -DVCI_NO_LIBRARY_INSTALL=ON
        -DOPENMESH_BUILD_SHARED=${OPENMESH_BUILD_SHARED}
	MAYBE_UNUSED_VARIABLES
		OPENMESH_BUILD_SHARED
        # [TODO]: add apps as feature, requires qt5 and freeglut
)

vcpkg_cmake_install()
vcpkg_copy_pdbs()
vcpkg_cmake_config_fixup(PACKAGE_NAME OpenMesh CONFIG_PATH "share/OpenMesh/cmake")
vcpkg_fixup_pkgconfig()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/include/OpenMesh/Tools/VDPM/xpm")

vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE")
