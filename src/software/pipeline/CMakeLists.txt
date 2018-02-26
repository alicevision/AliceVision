## AliceVision

## SfM softwares

# Intrinsic image analysis and SfMData container initialization

add_executable(aliceVision_cameraInit main_cameraInit.cpp)

target_link_libraries(aliceVision_cameraInit
  aliceVision_system
  aliceVision_sensorDB
  aliceVision_image
  aliceVision_feature
  aliceVision_sfm
  ${BOOST_LIBRARIES}
)

set_property(TARGET aliceVision_cameraInit
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_cameraInit
  DESTINATION bin/
)

# Feature extraction

add_executable(aliceVision_featureExtraction main_featureExtraction.cpp)

target_link_libraries(aliceVision_featureExtraction
  aliceVision_system
  aliceVision_image
  aliceVision_feature
  aliceVision_multiview
  aliceVision_sfm
  vlsift
  ${BOOST_LIBRARIES}
)

if(ALICEVISION_HAVE_CCTAG)
  target_link_libraries(aliceVision_featureExtraction CCTag::CCTag)
endif()

set_property(TARGET aliceVision_featureExtraction
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_featureExtraction
  DESTINATION bin/
)

# Image matching
# - generate the pair lists

add_executable(aliceVision_imageMatching main_imageMatching.cpp)

target_link_libraries(aliceVision_imageMatching
	aliceVision_voctree
	${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_imageMatching
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_imageMatching
	DESTINATION bin/
)

# Feature matching
# - putative matches and geometric filtered matches

add_executable(aliceVision_featureMatching main_featureMatching.cpp)

target_link_libraries(aliceVision_featureMatching
  aliceVision_system
  aliceVision_feature
  aliceVision_multiview
  aliceVision_sfm
  aliceVision_matchingImageCollection
  stlplus
  ${BOOST_LIBRARIES}
)

set_property(TARGET aliceVision_featureMatching
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_featureMatching
  DESTINATION bin/
)

# Incremental / Sequential SfM

add_executable(aliceVision_incrementalSfM main_incrementalSfM.cpp)

target_link_libraries(aliceVision_incrementalSfM
  aliceVision_system
  aliceVision_image
  aliceVision_feature
  aliceVision_sfm
  stlplus
  ${BOOST_LIBRARIES}
)

set_property(TARGET aliceVision_incrementalSfM
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_incrementalSfM
  DESTINATION bin/
)

# Global SfM

add_executable(aliceVision_globalSfM main_globalSfM.cpp)

target_link_libraries(aliceVision_globalSfM
  aliceVision_system
  aliceVision_image
  aliceVision_feature
  aliceVision_sfm
  stlplus
  ${BOOST_LIBRARIES}
)

set_property(TARGET aliceVision_globalSfM
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_globalSfM
  DESTINATION bin/
)

# Compute structure from known camera poses

add_executable(aliceVision_computeStructureFromKnownPoses main_computeStructureFromKnownPoses.cpp)

target_link_libraries(aliceVision_computeStructureFromKnownPoses
  aliceVision_system
  aliceVision_feature
  aliceVision_sfm
  stlplus
  ${BOOST_LIBRARIES}
)

set_property(TARGET aliceVision_computeStructureFromKnownPoses
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_computeStructureFromKnownPoses
  DESTINATION bin/
)

# Compute structure color

add_executable(aliceVision_computeSfMColor main_computeSfMColor.cpp)

target_link_libraries(aliceVision_computeSfMColor
  aliceVision_system
  aliceVision_image
  aliceVision_feature
  aliceVision_sfm
  stlplus
  ${BOOST_LIBRARIES}
)

set_property(TARGET aliceVision_computeSfMColor
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_computeSfMColor
  DESTINATION bin/
)

# Calibrate a camera 

if(ALICEVISION_HAVE_OPENCV)
  add_executable(aliceVision_cameraCalibration main_cameraCalibration.cpp)
  
  target_link_libraries(aliceVision_cameraCalibration # PUBLIC
    aliceVision_dataio
    aliceVision_image
    aliceVision_calibration
    aliceVision_system
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
  )

  if(ALICEVISION_HAVE_CCTAG)
    target_link_libraries(aliceVision_cameraCalibration CCTag::CCTag)
  endif()
  
  set_property(TARGET aliceVision_cameraCalibration
	PROPERTY FOLDER AliceVision/Software/Pipeline
  )

  install(TARGETS aliceVision_cameraCalibration 
	DESTINATION bin/
  )
endif()

# Calibrate a rig 

add_executable(aliceVision_rigCalibration main_rigCalibration.cpp)

target_link_libraries(aliceVision_rigCalibration
	aliceVision_rig
	aliceVision_localization
	aliceVision_dataio
	aliceVision_image
	aliceVision_feature
	vlsift  #TODO: delete vlsift
	${Boost_LIBRARIES}
)

if(ALICEVISION_HAVE_CCTAG)
	target_link_libraries(aliceVision_rigCalibration CCTag::CCTag)
endif()

set_property(TARGET aliceVision_rigCalibration
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_rigCalibration 
	DESTINATION bin/
)

# Localize a camera

add_executable(aliceVision_cameraLocalization main_cameraLocalization.cpp)

target_link_libraries(aliceVision_cameraLocalization
	aliceVision_localization
	aliceVision_dataio
	aliceVision_image
	aliceVision_feature
	vlsift
	${BOOST_LIBRARIES}
)

if(ALICEVISION_HAVE_ALEMBIC)
	target_link_libraries(aliceVision_cameraLocalization 
		aliceVision_sfm 
		${ABC_LIBRARIES}
	)
endif()

if(ALICEVISION_HAVE_CCTAG)
	target_link_libraries(aliceVision_cameraLocalization CCTag::CCTag)
endif()

set_property(TARGET aliceVision_cameraLocalization
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_cameraLocalization 
	DESTINATION bin/
)

# Localize a rig

add_executable(aliceVision_rigLocalization main_rigLocalization.cpp)

target_link_libraries(aliceVision_rigLocalization
	aliceVision_localization
	aliceVision_dataio
	aliceVision_rig
	aliceVision_image
	aliceVision_feature
	vlsift
	${Boost_LIBRARIES}
)

if(ALICEVISION_HAVE_ALEMBIC)
  target_link_libraries(aliceVision_rigLocalization aliceVision_sfm ${ABC_LIBRARIES})
endif()

if(ALICEVISION_HAVE_CCTAG)
  target_link_libraries(aliceVision_rigLocalization CCTag::CCTag)
endif()

set_property(TARGET aliceVision_rigLocalization
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_rigLocalization
	DESTINATION bin/
)

# Prepare dense scene for MVS

add_executable(aliceVision_prepareDenseScene main_prepareDenseScene.cpp)

target_link_libraries(aliceVision_prepareDenseScene
  aliceVision_system
  aliceVision_image
  aliceVision_feature
  aliceVision_sfm
  stlplus
  ${BOOST_LIBRARIES}
)

set_property(TARGET aliceVision_prepareDenseScene
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_prepareDenseScene
  DESTINATION bin/
)


## MVS software

# Compute Nearby Cameras

add_executable(aliceVision_cameraConnection main_cameraConnection.cpp)

target_link_libraries(aliceVision_cameraConnection
  PUBLIC aliceVision_common
         ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_cameraConnection
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_cameraConnection
  DESTINATION bin/
)

# Depth Map Estimation

if(ALICEVISION_HAVE_CUDA) # Depth map computation need CUDA
  add_executable(aliceVision_depthMapEstimation main_depthMapEstimation.cpp)

  target_link_libraries(aliceVision_depthMapEstimation
    PUBLIC aliceVision_structures
           aliceVision_common
           aliceVision_depthMap
           ${Boost_LIBRARIES}
  )

  set_property(TARGET aliceVision_depthMapEstimation
    PROPERTY FOLDER AliceVision/Software/Pipeline
  )

  install(TARGETS aliceVision_depthMapEstimation
    DESTINATION bin/
  )
endif()

# Depth Map Filtering

add_executable(aliceVision_depthMapFiltering main_depthMapFiltering.cpp)

target_link_libraries(aliceVision_depthMapFiltering
  PUBLIC aliceVision_structures
         aliceVision_common
         aliceVision_fuseCut
         ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_depthMapFiltering
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_depthMapFiltering
  DESTINATION bin/
)

# Meshing

add_executable(aliceVision_meshing main_meshing.cpp)

target_link_libraries(aliceVision_meshing
  PUBLIC aliceVision_structures
         aliceVision_common
         aliceVision_mesh
         aliceVision_fuseCut
         ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_meshing
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_meshing
  DESTINATION bin/
)

if(ALICEVISION_HAVE_MESHSDFILTER)
  
  # Mesh Denoising
  
  add_executable(aliceVision_meshDenoising main_meshDenoising.cpp)
  
  target_link_libraries(aliceVision_meshDenoising
    PUBLIC aliceVision_common
           MeshSDLibrary
           ${Boost_LIBRARIES}
           Eigen3::Eigen
  )
  
  set_property(TARGET aliceVision_meshDenoising
    PROPERTY FOLDER AliceVision/Software/Pipeline
  )
  
  install(TARGETS aliceVision_meshDenoising DESTINATION bin/)
  
  # Mesh Decimate
  
  add_executable(aliceVision_meshDecimate main_meshDecimate.cpp)
  
  target_link_libraries(aliceVision_meshDecimate
    PUBLIC aliceVision_common
           OpenMesh
           ${Boost_LIBRARIES}
  )
  
  set_property(TARGET aliceVision_meshDecimate
    PROPERTY FOLDER AliceVision/Software/Pipeline
  )
  
  install(TARGETS aliceVision_meshDecimate DESTINATION bin/)

endif()

# Mesh Filtering

add_executable(aliceVision_meshFiltering main_meshFiltering.cpp)

target_link_libraries(aliceVision_meshFiltering
  PUBLIC aliceVision_common
         aliceVision_mesh
         ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_meshFiltering
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_meshFiltering DESTINATION bin/)

# Mesh Resampling

add_executable(aliceVision_meshResampling main_meshResampling.cpp)

target_link_libraries(aliceVision_meshResampling
  PUBLIC aliceVision_common
         Geogram::geogram
         ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_meshResampling
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_meshResampling
  DESTINATION bin/
)

# Texturing

add_executable(aliceVision_texturing main_texturing.cpp)

target_link_libraries(aliceVision_texturing
  PUBLIC aliceVision_structures
         aliceVision_common
         aliceVision_mesh
         ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_texturing
  PROPERTY FOLDER AliceVision/Software/Pipeline
)

install(TARGETS aliceVision_texturing
  DESTINATION bin/
)


