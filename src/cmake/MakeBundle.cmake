include(BundleUtilities)

# Perform bundle fixup on all executables of an install directory
# and generates a standalone bundle with all required runtime dependencies.
#
# This scripts accepts the following parameters:
#   - CMAKE_INSTALL_PREFIX: install target path
#   - BUNDLE_INSTALL_PREFIX: bundle installation path
#   - BUNDLE_LIBS_PATHS: additional paths (colon separated) to look for runtime dependencies

message(STATUS "Starting Bundle")
message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "BUNDLE_INSTALL_PREFIX: ${BUNDLE_INSTALL_PREFIX}")
message(STATUS "BUNDLE_LIBS_PATHS: ${BUNDLE_LIBS_PATHS}")

# Add installed 'lib' folder to dependencies lookup path
set(LIBS_LOOKUPS_PATHS "${CMAKE_INSTALL_PREFIX}/lib")
list(APPEND LIBS_LOOKUPS_PATHS ${BUNDLE_LIBS_PATHS})

# Get all executables in installed 'bin' folder
# and copy them in the bundle installation path
get_bundle_all_executables(${CMAKE_INSTALL_PREFIX}/bin BUNDLE_APPS)
file(INSTALL ${BUNDLE_APPS} DESTINATION ${BUNDLE_INSTALL_PREFIX})

# Get first bundled executable as reference app
# fixup_bundle will automatically fixup all the others executable in the bundle
get_bundle_all_executables(${BUNDLE_INSTALL_PREFIX} BUNDLE_APPS)
list(GET BUNDLE_APPS 0 MAIN_APP)
fixup_bundle(${MAIN_APP} "" "${LIBS_LOOKUPS_PATHS}")

message(STATUS "Bundle done: ${BUNDLE_INSTALL_PREFIX}")
