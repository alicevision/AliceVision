include(BundleUtilities)

message(INFO "ALICEVISION_BUNDLE_PREFIX: ${ALICEVISION_BUNDLE_PREFIX}")
message(INFO "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")

get_bundle_all_executables(${CMAKE_INSTALL_PREFIX}/bin AV_APPS)
message(INFO "AV_APPS: ${AV_APPS}")

# Perform the bundle fixup
foreach(f ${AV_APPS})
  get_filename_component(filename "${f}" NAME)
  set(bf "${ALICEVISION_BUNDLE_PREFIX}/${filename}")
  message(INFO "Bundle application: ${f} => ${bf}")
  file(COPY "${f}" DESTINATION "${ALICEVISION_BUNDLE_PREFIX}")
  fixup_bundle("${bf}" "" "${CMAKE_INSTALL_PREFIX}/lib" )
  verify_app("${bf}")
endforeach()

