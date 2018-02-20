## AliceVision
## Pipeline software

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
