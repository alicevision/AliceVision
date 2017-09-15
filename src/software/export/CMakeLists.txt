## AliceVision
## Exportation software

# Export view extracted keypoints

add_executable(aliceVision_exportKeypoints main_exportKeypoints.cpp)

target_link_libraries(aliceVision_exportKeypoints
  aliceVision_system
  aliceVision_feature
  aliceVision_sfm
  stlplus
)

set_property(TARGET aliceVision_exportKeypoints
  PROPERTY FOLDER AliceVision/Software/Export
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
  stlplus
)

set_property(TARGET aliceVision_exportMatches
  PROPERTY FOLDER AliceVision/Software/Export
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
  stlplus
)

set_property(TARGET aliceVision_exportTracks
  PROPERTY FOLDER AliceVision/Software/Export
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
  stlplus
)

set_property(TARGET aliceVision_exportUndistortedImages
  PROPERTY FOLDER AliceVision/Software/Export
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
  stlplus
)

set_property(TARGET aliceVision_exportPMVS
  PROPERTY FOLDER AliceVision/Software/Export
)

install(TARGETS aliceVision_exportPMVS
  DESTINATION bin/
)

# Export a SfM aliceVision scene to CMPMVS format

add_executable(aliceVision_exportCMPMVS main_exportCMPMVS.cpp)

target_link_libraries(aliceVision_exportCMPMVS
  aliceVision_system
  aliceVision_image
  aliceVision_feature
  aliceVision_sfm
  stlplus
)

set_property(TARGET aliceVision_exportCMPMVS
  PROPERTY FOLDER AliceVision/Software/Export
)

install(TARGETS aliceVision_exportCMPMVS
  DESTINATION bin/
)

# Export a SfM aliceVision scene to CMPMVS format

add_executable(aliceVision_exportCMPMVS2 main_exportCMPMVS2.cpp)

target_link_libraries(aliceVision_exportCMPMVS2
  aliceVision_system
  aliceVision_image
  aliceVision_feature
  aliceVision_sfm
  stlplus
)

set_property(TARGET aliceVision_exportCMPMVS2
  PROPERTY FOLDER AliceVision/Software/Export
)

install(TARGETS aliceVision_exportCMPMVS2
  DESTINATION bin/
)

# Export a SfM aliceVision scene to MVE(v2) format

add_executable(aliceVision_exportMVE2 main_exportMVE2.cpp)

target_link_libraries(aliceVision_exportMVE2
  aliceVision_system
  aliceVision_image
  aliceVision_feature
  aliceVision_sfm
  stlplus
)

set_property(TARGET aliceVision_exportMVE2
  PROPERTY FOLDER AliceVision/Software/Export
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
  stlplus
)

set_property(TARGET aliceVision_exportMeshlab
  PROPERTY FOLDER AliceVision/Software/Export
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
  stlplus
)

set_property(TARGET aliceVision_exportMVSTexturing
  PROPERTY FOLDER AliceVision/Software/Export
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
  stlplus
)

set_property(TARGET aliceVision_exportMatlab
  PROPERTY FOLDER AliceVision/Software/Export
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
  stlplus
)

set_property(TARGET aliceVision_exportCameraFrustums
  PROPERTY FOLDER AliceVision/Software/Export
)

install(TARGETS aliceVision_exportCameraFrustums
  DESTINATION bin/
)
