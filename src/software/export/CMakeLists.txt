## AliceVision
## Exportation software

# Export an alembic animated camera from a SfMData file

if(ALICEVISION_HAVE_ALEMBIC)
  add_executable(aliceVision_exportAnimatedCamera main_exportAnimatedCamera.cpp)

  target_link_libraries(aliceVision_exportAnimatedCamera
    aliceVision_system
    aliceVision_sfm
    ${BOOST_LIBRARIES}
  )

  set_property(TARGET aliceVision_exportAnimatedCamera
    PROPERTY FOLDER AliceVision/Software/Export
  )

  install(TARGETS aliceVision_exportAnimatedCamera
    DESTINATION bin/
  )
endif()

# Export view extracted keypoints

add_executable(aliceVision_exportKeypoints main_exportKeypoints.cpp)

target_link_libraries(aliceVision_exportKeypoints
  aliceVision_system
  aliceVision_feature
  aliceVision_sfm
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_exportKeypoints
  PROPERTY FOLDER Software/Export
)

install(TARGETS aliceVision_exportKeypoints
  DESTINATION bin/
)

# Export view computed matches (putatives, geometrics) per image pair

add_executable(aliceVision_exportMatches main_exportMatches.cpp)

target_link_libraries(aliceVision_exportMatches
  aliceVision_system
  aliceVision_feature
  aliceVision_sfm
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_exportMatches
  PROPERTY FOLDER Software/Export
)

install(TARGETS aliceVision_exportMatches
  DESTINATION bin/
)

# Export view tracks per image pair

add_executable(aliceVision_exportTracks main_exportTracks.cpp)

target_link_libraries(aliceVision_exportTracks
  aliceVision_system
  aliceVision_feature
  aliceVision_sfm
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_exportTracks
  PROPERTY FOLDER Software/Export
)

install(TARGETS aliceVision_exportTracks
  DESTINATION bin/
)

# Export undistorted images related to a sfmData file

add_executable(aliceVision_exportUndistortedImages main_exportUndistortedImages.cpp)

target_link_libraries(aliceVision_exportUndistortedImages
  aliceVision_system
  aliceVision_image
  aliceVision_feature
  aliceVision_sfm
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_exportUndistortedImages
  PROPERTY FOLDER Software/Export
)

install(TARGETS aliceVision_exportUndistortedImages
  DESTINATION bin/
)

# Export a SfM aliceVision scene to PMVS format

add_executable(aliceVision_exportPMVS main_exportPMVS.cpp)

target_link_libraries(aliceVision_exportPMVS
  aliceVision_system
  aliceVision_image
  aliceVision_feature
  aliceVision_sfm
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_exportPMVS
  PROPERTY FOLDER Software/Export
)

install(TARGETS aliceVision_exportPMVS
  DESTINATION bin/
)

# Export a SfM aliceVision scene to MayaMVG

if(ALICEVISION_HAVE_ALEMBIC) # maya can read alembic file
  add_executable(aliceVision_exportMayaMVG main_exportMayaMVG.cpp)

  target_link_libraries(aliceVision_exportMayaMVG
    aliceVision_system
    aliceVision_image
    aliceVision_feature
    aliceVision_sfm
    ${Boost_LIBRARIES}
    ${OPENIMAGEIO_LIBRARIES}
  )

  set_property(TARGET aliceVision_exportMayaMVG
    PROPERTY FOLDER Software/Export
  )

  install(TARGETS aliceVision_exportMayaMVG
    DESTINATION bin/
  )
endif()

# Export a SfM aliceVision scene to MVE(v2) format

add_executable(aliceVision_exportMVE2 main_exportMVE2.cpp)

target_link_libraries(aliceVision_exportMVE2
  aliceVision_system
  aliceVision_image
  aliceVision_feature
  aliceVision_sfm
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_exportMVE2
  PROPERTY FOLDER Software/Export
)

install(TARGETS aliceVision_exportMVE2
  DESTINATION bin/
)

# Export a SfM aliceVision scene to meshlab scene with rasters

add_executable(aliceVision_exportMeshlab main_exportMeshlab.cpp)

target_link_libraries(aliceVision_exportMeshlab
  aliceVision_system
  aliceVision_image
  aliceVision_feature
  aliceVision_sfm
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_exportMeshlab
  PROPERTY FOLDER Software/Export
)

install(TARGETS aliceVision_exportMeshlab
  DESTINATION bin/
)

# Export a SfM aliceVision scene to mvs-texturing scene folder

add_executable(aliceVision_exportMVSTexturing main_exportMVSTexturing.cpp)

target_link_libraries(aliceVision_exportMVSTexturing
  aliceVision_system
  aliceVision_image
  aliceVision_feature
  aliceVision_sfm
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_exportMVSTexturing
  PROPERTY FOLDER Software/Export
)

install(TARGETS aliceVision_exportMVSTexturing
  DESTINATION bin/
)

# Export a SfM aliceVision scene to Matlab

add_executable(aliceVision_exportMatlab main_exportMatlab.cpp)

target_link_libraries(aliceVision_exportMatlab
  aliceVision_system
  aliceVision_image
  aliceVision_feature
  aliceVision_sfm
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_exportMatlab
  PROPERTY FOLDER Software/Export
)

install(TARGETS aliceVision_exportMatlab
  DESTINATION bin/
)

# Export SfM aliceVision camera scene as triangle meshes

add_executable(aliceVision_exportCameraFrustums main_exportCameraFrustums.cpp)

target_link_libraries(aliceVision_exportCameraFrustums
  aliceVision_system
  aliceVision_feature
  aliceVision_sfm
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_exportCameraFrustums
  PROPERTY FOLDER Software/Export
)

install(TARGETS aliceVision_exportCameraFrustums
  DESTINATION bin/
)
