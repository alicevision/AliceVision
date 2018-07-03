include(BundleUtilities)

# Perform bundle fixup on all executables of an install directory
# and generates a standalone bundle with all required runtime dependencies.
# 
# This scripts accepts the following parameters:
#   - CMAKE_INSTALL_PREFIX: install target path
#   - BUNDLE_INSTALL_PREFIX: bundle installation path
#   - BUNDLE_LIBS_PATHS: additional paths (semi-colon separated) to look for runtime dependencies 
message(STATUS "Starting Bundle")
message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
message(STATUS "BUNDLE_INSTALL_PREFIX: ${BUNDLE_INSTALL_PREFIX}")
message(STATUS "BUNDLE_LIBS_PATHS: ${BUNDLE_LIBS_PATHS}")

# Add installed 'lib' folder to dependencies lookup path
set(LIBS_LOOKUPS_PATHS "${CMAKE_INSTALL_PREFIX}/lib")
list(APPEND LIBS_LOOKUPS_PATHS ${BUNDLE_LIBS_PATHS})

# Get all executables in installed 'bin' folder
get_bundle_all_executables(${CMAKE_INSTALL_PREFIX}/bin BUNDLE_APPS)
message(STATUS "BUNDLE_APPS: ${BUNDLE_APPS}")

# Perform the bundle fixup
foreach(f ${BUNDLE_APPS})
  get_filename_component(filename "${f}" NAME)
  set(bf "${BUNDLE_INSTALL_PREFIX}/${filename}")
  message(STATUS "Bundle application: ${f} => ${bf}")
  file(COPY "${f}" DESTINATION "${BUNDLE_INSTALL_PREFIX}")
  fixup_bundle(${bf} "" "${LIBS_LOOKUPS_PATHS}" )
  verify_app("${bf}")
endforeach()

message(STATUS "Bundle done")
