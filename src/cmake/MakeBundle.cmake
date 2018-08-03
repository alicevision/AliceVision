# Perform bundle fixup on all executables of an install directory
# and generates a standalone bundle with all required runtime dependencies.
#
# This scripts accepts the following parameters:
#   - CMAKE_INSTALL_PREFIX: install target path
#   - BUNDLE_INSTALL_PREFIX: bundle installation path
#   - BUNDLE_LIBS_PATHS: additional paths (colon separated) to look for runtime dependencies

function(gp_resolve_item_override context item exepath dirs resolved_item_var resolved_var)
  # avoid log flood for those system libraries with non-absolute path
  if(item MATCHES "^(api-ms-win-)[^/]+dll")
    # resolve item with fake absolute system path to keep them identified as system libs
    # By doing this, fixup_bundle:
    #   - won't complain about those libraries
    #   - won't embed them in the bundle
    set(${resolved_item_var} "$ENV{SystemRoot}/system/${item}" PARENT_SCOPE)
    set(${resolved_var} TRUE PARENT_SCOPE)
  endif()
endfunction()

include(BundleUtilities)
include(GNUInstallDirs)

message(STATUS "Starting Bundle")
message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "BUNDLE_INSTALL_PREFIX: ${BUNDLE_INSTALL_PREFIX}")
message(STATUS "BUNDLE_LIBS_PATHS: ${BUNDLE_LIBS_PATHS}")

# message(STATUS "CMAKE_INSTALL_FULL_LIBDIR: ${CMAKE_INSTALL_FULL_LIBDIR}")
# message(STATUS "CMAKE_INSTALL_LIBDIR: ${CMAKE_INSTALL_LIBDIR}")

# Add installed runtime library folder to dependencies lookup path
if(WIN32)  # installed next to binaries on Windows
    set(LIBS_LOOKUPS_PATHS "${CMAKE_INSTALL_FULL_BINDIR}")
else()     # installed in library dir everywhere else
    set(LIBS_LOOKUPS_PATHS "${CMAKE_INSTALL_FULL_LIBDIR}")
    # GNUInstallDirs is not able to resolve between lib and lib64 hen cmake is called as a sub-command line.
    # As a workaround we always add a second path with "64" suffix, so it works in all cases.
    # In some cases, that will be useless and point to a non-existing directory.
    list(APPEND LIBS_LOOKUPS_PATHS ${CMAKE_INSTALL_FULL_LIBDIR}64)
endif()

if(BUNDLE_LIBS_PATHS)
list(APPEND LIBS_LOOKUPS_PATHS ${BUNDLE_LIBS_PATHS})
endif()
message(STATUS "LIBS_LOOKUPS_PATHS: ${LIBS_LOOKUPS_PATHS}")

# Get all executables in installed 'bin' folder
# and copy them in the bundle installation path
get_bundle_all_executables(${CMAKE_INSTALL_FULL_BINDIR} BUNDLE_APPS)
file(INSTALL ${BUNDLE_APPS} DESTINATION ${BUNDLE_INSTALL_PREFIX} USE_SOURCE_PERMISSIONS)

# Get first bundled executable as reference app
# fixup_bundle will automatically fixup all the others executable in the bundle
get_bundle_all_executables(${BUNDLE_INSTALL_PREFIX} BUNDLE_APPS)
list(GET BUNDLE_APPS 0 MAIN_APP)
fixup_bundle(${MAIN_APP} "" "${LIBS_LOOKUPS_PATHS}")

message(STATUS "Bundle done: ${BUNDLE_INSTALL_PREFIX}")

