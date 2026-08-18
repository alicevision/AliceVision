set(VCPKG_TARGET_ARCHITECTURE x64)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE dynamic)
set(VCPKG_BUILD_TYPE release)
set(VCPKG_PROVIDED_FORTRAN ON)

if(PORT STREQUAL "onnx")
    # simply set to use single flag or list append to customize multiple flags
    list(APPEND VCPKG_CMAKE_CONFIGURE_OPTIONS "-DONNX_DISABLE_STATIC_REGISTRATION=ON")
endif()