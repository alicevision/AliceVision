#the port uses inside the CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS, which is discouraged by vcpkg.

# Note: AliceVision fork = upstream f9caaf6 + "fix random seed" (KDTreeIndex honours the "random_seed" index param),
# same source as DEP_FLANN_GIT_TAG in src/cmake/DependenciesVersions.cmake
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO alicevision/flann
    REF  46e72429ef60ce9c413fa926ac7729f8dee96395
    SHA512 7316b2d4649a1ff417dab7cf5471fbb6cd73f987b8171762ccb328996836b4ec460d27c65c15ec531265cf1bc76407a95c8c4c848a88e0b36ed52d5e261ec542
    HEAD_REF master
    PATCHES
        fix-dependency-hdf5.patch
        fix-dep-lz4.patch
)

vcpkg_check_features(OUT_FEATURE_OPTIONS FEATURE_OPTIONS
    FEATURES
        hdf5    WITH_HDF5
        cuda    BUILD_CUDA_LIB
)
if(VCPKG_LIBRARY_LINKAGE STREQUAL "static")
   set(LINK_STATIC ON)
else()
   set(LINK_STATIC OFF)
endif()

vcpkg_cmake_configure(
    SOURCE_PATH ${SOURCE_PATH}
    DISABLE_PARALLEL_CONFIGURE
    OPTIONS ${FEATURE_OPTIONS}
        -DBUILD_EXAMPLES=OFF
        -DBUILD_TESTS=OFF
        -DBUILD_DOC=OFF
        -DBUILD_PYTHON_BINDINGS=OFF
        -DBUILD_MATLAB_BINDINGS=OFF
        -DUSE_OPENMP=OFF
        -DCMAKE_BUILD_STATIC_LIBS=${LINK_STATIC}
    OPTIONS_DEBUG 
        -DCMAKE_DEBUG_POSTFIX=d
)

vcpkg_cmake_install()
vcpkg_cmake_config_fixup(CONFIG_PATH lib/cmake/${PORT})

file(REMOVE_RECURSE ${CURRENT_PACKAGES_DIR}/debug/include)
# Handle copyright
vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/COPYING")

vcpkg_fixup_pkgconfig()
