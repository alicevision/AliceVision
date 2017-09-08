###
# Intrinsic image analysis and SfM_Data container initialization
###
add_executable(aliceVision_main_SfMInit_ImageListing main_SfMInit_ImageListing.cpp)

target_link_libraries(aliceVision_main_SfMInit_ImageListing
  aliceVision_system
  aliceVision_image
  aliceVision_features
  aliceVision_exif
  aliceVision_sfm
  easyexif
)

#convert a v0.6 lists.txt file to the new sfm_data.X format
add_executable(aliceVision_main_ConvertList main_ConvertList.cpp)

target_link_libraries(aliceVision_main_ConvertList
  aliceVision_system
  aliceVision_features
  aliceVision_sfm
)

# Installation rules
set_property(TARGET aliceVision_main_SfMInit_ImageListing
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_SfMInit_ImageListing
  DESTINATION bin/
)

set_property(TARGET aliceVision_main_ConvertList
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_ConvertList
  DESTINATION bin/
)

###
# Add executable that computes:
# - aliceVision_main_ComputeFeatures: features and descriptors
# - aliceVision_main_ComputeMatches: putative matches + geometric filtered matches
###

add_executable(aliceVision_main_ComputeFeatures main_ComputeFeatures.cpp)

target_link_libraries(aliceVision_main_ComputeFeatures
  aliceVision_system
  aliceVision_image
  aliceVision_features
  aliceVision_multiview
  aliceVision_sfm
  aliceVision_exif
  easyexif
  stlplus
  vlsift
)

if(ALICEVISION_HAVE_CCTAG)
  target_link_libraries(aliceVision_main_ComputeFeatures CCTag::CCTag)
endif()

add_executable(aliceVision_main_ComputeMatches main_ComputeMatches.cpp)

target_link_libraries(aliceVision_main_ComputeMatches
  aliceVision_system
  aliceVision_features
  aliceVision_multiview
  aliceVision_sfm
  aliceVision_matching_image_collection
  stlplus
)

# Installation rules
set_property(TARGET aliceVision_main_ComputeFeatures
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_ComputeFeatures
  DESTINATION bin/
)

set_property(TARGET aliceVision_main_ComputeMatches
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_ComputeMatches
  DESTINATION bin/
)

###
# SfM Pipelines
# - Incremental/Sequential
# - Global
# - Convert SfM_Data format (from one to another)
# - Pair filtering (from frustum)
# - Compute structure from known camera poses
# - Compute structure color
###
add_executable(aliceVision_main_IncrementalSfM main_IncrementalSfM.cpp)

target_link_libraries(aliceVision_main_IncrementalSfM
  aliceVision_system
  aliceVision_image
  aliceVision_features
  aliceVision_sfm
  stlplus
)

add_executable(aliceVision_main_GlobalSfM main_GlobalSfM.cpp)

target_link_libraries(aliceVision_main_GlobalSfM
  aliceVision_system
  aliceVision_image
  aliceVision_features
  aliceVision_sfm
  stlplus
)

add_executable(aliceVision_main_ConvertSfM_DataFormat main_ConvertSfM_DataFormat.cpp)

target_link_libraries(aliceVision_main_ConvertSfM_DataFormat
  aliceVision_system
  aliceVision_features
  aliceVision_sfm
  stlplus
)

if(Boost_FOUND)
  target_link_libraries(aliceVision_main_ConvertSfM_DataFormat ${Boost_LIBRARIES})
endif()

add_executable(aliceVision_main_Align main_Align.cpp)

target_link_libraries(aliceVision_main_Align
  aliceVision_system
  aliceVision_features
  aliceVision_sfm
  stlplus
)

add_executable(aliceVision_main_FrustumFiltering main_FrustumFiltering.cpp)

target_link_libraries(aliceVision_main_FrustumFiltering
  aliceVision_system
  aliceVision_features
  aliceVision_sfm
  stlplus
)

add_executable(aliceVision_main_ComputeStructureFromKnownPoses main_ComputeStructureFromKnownPoses.cpp)

target_link_libraries(aliceVision_main_ComputeStructureFromKnownPoses
  aliceVision_system
  aliceVision_features
  aliceVision_sfm
  stlplus
)

add_executable(aliceVision_main_ComputeSfM_DataColor main_ComputeSfM_DataColor.cpp)

target_link_libraries(aliceVision_main_ComputeSfM_DataColor
  aliceVision_system
  aliceVision_image
  aliceVision_features
  aliceVision_sfm
  stlplus
)

if(ALICEVISION_HAVE_ALEMBIC)
  add_executable(aliceVision_main_ConvertAnimatedCamera main_ConvertAnimatedCamera.cpp)

  target_link_libraries(aliceVision_main_ConvertAnimatedCamera
    aliceVision_system
    aliceVision_features
    aliceVision_sfm
    stlplus
  )
endif()

# Installation rules
set_property(TARGET aliceVision_main_IncrementalSfM
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_IncrementalSfM
  DESTINATION bin/
)

set_property(TARGET aliceVision_main_GlobalSfM
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_GlobalSfM
  DESTINATION bin/
)

set_property(TARGET aliceVision_main_ConvertSfM_DataFormat
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_ConvertSfM_DataFormat
  DESTINATION bin/
)

set_property(TARGET aliceVision_main_Align
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_Align
  DESTINATION bin/
)

set_property(TARGET aliceVision_main_FrustumFiltering
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_FrustumFiltering
  DESTINATION bin/
)

set_property(TARGET aliceVision_main_ComputeStructureFromKnownPoses
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_ComputeStructureFromKnownPoses
  DESTINATION bin/
)

set_property(TARGET aliceVision_main_ComputeSfM_DataColor
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_ComputeSfM_DataColor
  DESTINATION bin/
)

if(ALICEVISION_HAVE_ALEMBIC)
  set_property(TARGET aliceVision_main_ConvertAnimatedCamera
    PROPERTY FOLDER AliceVision/software
  )
  install(TARGETS aliceVision_main_ConvertAnimatedCamera
    DESTINATION bin/
  )
endif()

###
# SfM tools to visualize feature tracking data
###

# - View extracted Keypoints
#
add_executable(aliceVision_main_exportKeypoints main_exportKeypoints.cpp)

target_link_libraries(aliceVision_main_exportKeypoints
  aliceVision_system
  aliceVision_features
  aliceVision_sfm
  stlplus
)


# - View computed matches (putatives, geometrics) per image pair
#
add_executable(aliceVision_main_exportMatches main_exportMatches.cpp)

target_link_libraries(aliceVision_main_exportMatches
  aliceVision_system
  aliceVision_features
  aliceVision_sfm
  stlplus
)

# - View tracks per image pair
#
add_executable(aliceVision_main_exportTracks main_exportTracks.cpp)

target_link_libraries(aliceVision_main_exportTracks
  aliceVision_system
  aliceVision_features
  aliceVision_sfm
  stlplus
)

# - Export undistorted images related to a sfm_data file
#
add_executable(aliceVision_main_ExportUndistortedImages main_ExportUndistortedImages.cpp)

target_link_libraries(aliceVision_main_ExportUndistortedImages
  aliceVision_system
  aliceVision_image
  aliceVision_features
  aliceVision_sfm
  stlplus
)

# installation rules
set_property(TARGET aliceVision_main_exportKeypoints
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_exportKeypoints
  DESTINATION bin/
)

set_property(TARGET aliceVision_main_exportMatches
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_exportMatches
  DESTINATION bin/
)

set_property(TARGET aliceVision_main_exportTracks
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_exportTracks
  DESTINATION bin/
)

set_property(TARGET aliceVision_main_ExportUndistortedImages
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_ExportUndistortedImages
  DESTINATION bin/
)

###
# SfM export to X
# - PMVS + bundler data format
# - CMPMVS
# - Meshlab
# - MVE (File format v2)
###

# - Export a SfM aliceVision scene to PMVS format
#
add_executable(aliceVision_main_aliceVision2PMVS main_aliceVision2PMVS.cpp)

target_link_libraries(aliceVision_main_aliceVision2PMVS
  aliceVision_system
  aliceVision_image
  aliceVision_features
  aliceVision_sfm
  stlplus
)

# - Export a SfM aliceVision scene to CMPMVS format
#
add_executable(aliceVision_main_aliceVision2CMPMVS main_aliceVision2CMPMVS.cpp)

target_link_libraries(aliceVision_main_aliceVision2CMPMVS
  aliceVision_system
  aliceVision_image
  aliceVision_features
  aliceVision_sfm
  stlplus
)

# - Export a SfM aliceVision scene to CMPMVS format
#
add_executable(aliceVision_main_aliceVision2CMPMVS2 main_aliceVision2CMPMVS2.cpp)

target_link_libraries(aliceVision_main_aliceVision2CMPMVS2
  aliceVision_system
  aliceVision_image
  aliceVision_features
  aliceVision_sfm
  stlplus
)


# - Export a SfM aliceVision scene to MVE(v2) format
#
add_executable(aliceVision_main_aliceVision2MVE2 main_aliceVision2MVE2.cpp)

target_link_libraries(aliceVision_main_aliceVision2MVE2
  aliceVision_system
  aliceVision_image
  aliceVision_features
  aliceVision_sfm
  stlplus
)

# - Export a SfM aliceVision scene to meshlab scene with rasters
# -
add_executable(aliceVision_main_aliceVision2MESHLAB main_aliceVision2MESHLAB.cpp)

target_link_libraries(aliceVision_main_aliceVision2MESHLAB
  aliceVision_system
  aliceVision_image
  aliceVision_features
  aliceVision_sfm
  stlplus
)

# - Export a SfM aliceVision scene to mvs-texturing scene folder
# -
add_executable(aliceVision_main_aliceVision2MVSTEXTURING main_aliceVision2MVSTEXTURING.cpp)

target_link_libraries(aliceVision_main_aliceVision2MVSTEXTURING
  aliceVision_system
  aliceVision_image
  aliceVision_features
  aliceVision_sfm
  stlplus
)

# - Export a SfM aliceVision scene to Matlab
#
add_executable(aliceVision_main_aliceVision2Matlab main_aliceVision2Matlab.cpp)
target_link_libraries(aliceVision_main_aliceVision2Matlab
  aliceVision_system
  aliceVision_image
  aliceVision_features
  aliceVision_sfm
  stlplus
  )


# - Export SfM aliceVision camera scene as triangle meshes
# -
add_executable(aliceVision_main_ExportCameraFrustums main_ExportCameraFrustums.cpp)

target_link_libraries(aliceVision_main_ExportCameraFrustums
  aliceVision_system
  aliceVision_features
  aliceVision_sfm
  stlplus
)

# installation rules
set_property(TARGET aliceVision_main_aliceVision2PMVS
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_aliceVision2PMVS
  DESTINATION bin/
)

set_property(TARGET aliceVision_main_aliceVision2CMPMVS
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_aliceVision2CMPMVS
  DESTINATION bin/
)

set_property(TARGET aliceVision_main_aliceVision2CMPMVS2
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_aliceVision2CMPMVS2
  DESTINATION bin/
)

set_property(TARGET aliceVision_main_aliceVision2MESHLAB
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_aliceVision2MESHLAB
  DESTINATION bin/
)

set_property(TARGET aliceVision_main_aliceVision2MVSTEXTURING
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_aliceVision2MVSTEXTURING
  DESTINATION bin/
)

set_property(TARGET aliceVision_main_aliceVision2Matlab
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_aliceVision2Matlab
  DESTINATION bin/)

set_property(TARGET aliceVision_main_aliceVision2MVE2
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_aliceVision2MVE2
  DESTINATION bin/
)

set_property(TARGET aliceVision_main_ExportCameraFrustums
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_ExportCameraFrustums
  DESTINATION bin/
)

###
# SfM Research tools
#
# - Quality comparison against a GT camera path (MultiView Evaluation dataset)
###
add_executable(aliceVision_main_evalQuality main_evalQuality.cpp)

target_link_libraries(aliceVision_main_evalQuality
  aliceVision_system
  aliceVision_features
  aliceVision_sfm
  stlplus
)

#installation rules
set_property(TARGET aliceVision_main_evalQuality
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_evalQuality
  DESTINATION bin/
)


###
# SfM tools to prepare picture to a future SfM process
###

# - Export keyframes from video files / image sequence directories
# -
if(ALICEVISION_HAVE_BOOST AND ALICEVISION_HAVE_OIIO)

add_executable(aliceVision_main_keyframeSelection main_keyframeSelection.cpp)
target_link_libraries(aliceVision_main_keyframeSelection
  aliceVision_system
  aliceVision_keyframe
  ${OPENIMAGEIO_LIBRARIES}
  ${Boost_LIBRARIES}
)

#installation rules
set_property(TARGET aliceVision_main_keyframeSelection
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_keyframeSelection
  DESTINATION bin/
)

endif()


##
# Export tutorial with valid path thanks to a configuration file
##
set(ALICEVISION_SOFTWARE_SFM_SRC_DIR ${CMAKE_CURRENT_LIST_DIR})
if(MSVC OR APPLE)
  set(ALICEVISION_SOFTWARE_SFM_BUILD_DIR "${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}")
else()
  set(ALICEVISION_SOFTWARE_SFM_BUILD_DIR ${EXECUTABLE_OUTPUT_PATH})
endif()

set(ALICEVISION_TUTORIAL_IN_FILE "${CMAKE_CURRENT_LIST_DIR}/tutorial_demo.py.in")
configure_file(${ALICEVISION_TUTORIAL_IN_FILE}
    "${CMAKE_CURRENT_BINARY_DIR}/tutorial_demo.py" @ONLY)

set(ALICEVISION_TUTORIAL_IN_FILE "${CMAKE_CURRENT_LIST_DIR}/SfM_Pipeline.py.in")
configure_file(${ALICEVISION_TUTORIAL_IN_FILE}
    "${CMAKE_CURRENT_BINARY_DIR}/SfM_Pipeline.py" @ONLY)
