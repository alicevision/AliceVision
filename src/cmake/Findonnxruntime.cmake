# This will define the following variables:
# onnxruntime_FOUND        -- True if the system has the onnxruntime library
# onnxruntime_INCLUDE_DIRS -- The include directories for onnxruntime
# onnxruntime_LIBRARIES    -- Libraries to link against
# onnxruntime_CXX_FLAGS    -- Additional (required) compiler flags

include(FindPackageHandleStandardArgs)

set(ONNXRUNTIME_SEARCH_PATHS
    ${ONNXRUNTIME_INSTALL_PREFIX}
    "$ENV{ONNXRUNTIME_INSTALL_PREFIX}"
    "/usr/local/"
)

find_path(onnxruntime_INSTALL_PREFIX
    PATHS ${ONNXRUNTIME_SEARCH_PATHS}
)

set(onnxruntime_INCLUDE_DIRS ${onnxruntime_INSTALL_PREFIX}/include)
set(onnxruntime_LIBRARIES onnxruntime)
set(onnxruntime_CXX_FLAGS "") # no flags needed

find_library(onnxruntime_LIBRARY onnxruntime
    PATHS "${onnxruntime_INSTALL_PREFIX}/lib"
)

add_library(onnxruntime SHARED IMPORTED)
set_property(TARGET onnxruntime PROPERTY IMPORTED_LOCATION "${onnxruntime_LIBRARY}")
set_property(TARGET onnxruntime PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_INCLUDE_DIRS}")
set_property(TARGET onnxruntime PROPERTY INTERFACE_COMPILE_OPTIONS "${onnxruntime_CXX_FLAGS}")

find_package_handle_standard_args(onnxruntime DEFAULT_MSG onnxruntime_LIBRARY onnxruntime_INCLUDE_DIRS)