###
# Intrinsic image analysis and SfM_Data container initialization
###
add_executable(openMVG_main_SfMInit_ImageListing main_SfMInit_ImageListing.cpp)

target_link_libraries(openMVG_main_SfMInit_ImageListing
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_exif
  openMVG_sfm
  easyexif
)

#convert a v0.6 lists.txt file to the new sfm_data.X format
add_executable(openMVG_main_ConvertList main_ConvertList.cpp)

target_link_libraries(openMVG_main_ConvertList
  openMVG_system
  openMVG_features
  openMVG_sfm
)

# Installation rules
set_property(TARGET openMVG_main_SfMInit_ImageListing
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_SfMInit_ImageListing
  DESTINATION bin/
)

set_property(TARGET openMVG_main_ConvertList
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_ConvertList
  DESTINATION bin/
)

###
# Add executable that computes:
# - openMVG_main_ComputeFeatures: features and descriptors
# - openMVG_main_ComputeMatches: putative matches + geometric filtered matches
###

add_executable(openMVG_main_ComputeFeatures main_ComputeFeatures.cpp)

target_link_libraries(openMVG_main_ComputeFeatures
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_multiview
  openMVG_sfm
  openMVG_exif
  easyexif
  stlplus
  vlsift
)

if(OPENMVG_HAVE_CCTAG)
  target_link_libraries(openMVG_main_ComputeFeatures CCTag::CCTag)
endif()

add_executable(openMVG_main_ComputeMatches main_ComputeMatches.cpp)

target_link_libraries(openMVG_main_ComputeMatches
  openMVG_system
  openMVG_features
  openMVG_multiview
  openMVG_sfm
  openMVG_matching_image_collection
  stlplus
)

# Installation rules
set_property(TARGET openMVG_main_ComputeFeatures
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_ComputeFeatures
  DESTINATION bin/
)

set_property(TARGET openMVG_main_ComputeMatches
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_ComputeMatches
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
add_executable(openMVG_main_IncrementalSfM main_IncrementalSfM.cpp)

target_link_libraries(openMVG_main_IncrementalSfM
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_sfm
  stlplus
)

add_executable(openMVG_main_GlobalSfM main_GlobalSfM.cpp)

target_link_libraries(openMVG_main_GlobalSfM
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_sfm
  stlplus
)

add_executable(openMVG_main_ConvertSfM_DataFormat main_ConvertSfM_DataFormat.cpp)

target_link_libraries(openMVG_main_ConvertSfM_DataFormat
  openMVG_system
  openMVG_features
  openMVG_sfm
  stlplus
)

if(Boost_FOUND)
  target_link_libraries(openMVG_main_ConvertSfM_DataFormat ${Boost_LIBRARIES})
endif()

add_executable(openMVG_main_Align main_Align.cpp)

target_link_libraries(openMVG_main_Align
  openMVG_system
  openMVG_features
  openMVG_sfm
  stlplus
)

add_executable(openMVG_main_FrustumFiltering main_FrustumFiltering.cpp)

target_link_libraries(openMVG_main_FrustumFiltering
  openMVG_system
  openMVG_features
  openMVG_sfm
  stlplus
)

add_executable(openMVG_main_ComputeStructureFromKnownPoses main_ComputeStructureFromKnownPoses.cpp)

target_link_libraries(openMVG_main_ComputeStructureFromKnownPoses
  openMVG_system
  openMVG_features
  openMVG_sfm
  stlplus
)

add_executable(openMVG_main_ComputeSfM_DataColor main_ComputeSfM_DataColor.cpp)

target_link_libraries(openMVG_main_ComputeSfM_DataColor
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_sfm
  stlplus
)

if(OPENMVG_HAVE_ALEMBIC)
  add_executable(openMVG_main_ConvertAnimatedCamera main_ConvertAnimatedCamera.cpp)

  target_link_libraries(openMVG_main_ConvertAnimatedCamera
    openMVG_system
    openMVG_features
    openMVG_sfm
    stlplus
  )
endif()

# Installation rules
set_property(TARGET openMVG_main_IncrementalSfM
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_IncrementalSfM
  DESTINATION bin/
)

set_property(TARGET openMVG_main_GlobalSfM
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_GlobalSfM
  DESTINATION bin/
)

set_property(TARGET openMVG_main_ConvertSfM_DataFormat
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_ConvertSfM_DataFormat
  DESTINATION bin/
)

set_property(TARGET openMVG_main_Align
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_Align
  DESTINATION bin/
)

set_property(TARGET openMVG_main_FrustumFiltering
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_FrustumFiltering
  DESTINATION bin/
)

set_property(TARGET openMVG_main_ComputeStructureFromKnownPoses
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_ComputeStructureFromKnownPoses
  DESTINATION bin/
)

set_property(TARGET openMVG_main_ComputeSfM_DataColor
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_ComputeSfM_DataColor
  DESTINATION bin/
)

if(OPENMVG_HAVE_ALEMBIC)
  set_property(TARGET openMVG_main_ConvertAnimatedCamera
    PROPERTY FOLDER OpenMVG/software
  )
  install(TARGETS openMVG_main_ConvertAnimatedCamera
    DESTINATION bin/
  )
endif()

###
# SfM tools to visualize feature tracking data
###

# - View extracted Keypoints
#
add_executable(openMVG_main_exportKeypoints main_exportKeypoints.cpp)

target_link_libraries(openMVG_main_exportKeypoints
  openMVG_system
  openMVG_features
  openMVG_sfm
  stlplus
)


# - View computed matches (putatives, geometrics) per image pair
#
add_executable(openMVG_main_exportMatches main_exportMatches.cpp)

target_link_libraries(openMVG_main_exportMatches
  openMVG_system
  openMVG_features
  openMVG_sfm
  stlplus
)

# - View tracks per image pair
#
add_executable(openMVG_main_exportTracks main_exportTracks.cpp)

target_link_libraries(openMVG_main_exportTracks
  openMVG_system
  openMVG_features
  openMVG_sfm
  stlplus
)

# - Export undistorted images related to a sfm_data file
#
add_executable(openMVG_main_ExportUndistortedImages main_ExportUndistortedImages.cpp)

target_link_libraries(openMVG_main_ExportUndistortedImages
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_sfm
  stlplus
)

# installation rules
set_property(TARGET openMVG_main_exportKeypoints
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_exportKeypoints
  DESTINATION bin/
)

set_property(TARGET openMVG_main_exportMatches
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_exportMatches
  DESTINATION bin/
)

set_property(TARGET openMVG_main_exportTracks
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_exportTracks
  DESTINATION bin/
)

set_property(TARGET openMVG_main_ExportUndistortedImages
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_ExportUndistortedImages
  DESTINATION bin/
)

###
# SfM export to X
# - PMVS + bundler data format
# - CMPMVS
# - Meshlab
# - MVE (File format v2)
###

# - Export a SfM openMVG scene to PMVS format
#
add_executable(openMVG_main_openMVG2PMVS main_openMVG2PMVS.cpp)

target_link_libraries(openMVG_main_openMVG2PMVS
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_sfm
  stlplus
)

# - Export a SfM openMVG scene to CMPMVS format
#
add_executable(openMVG_main_openMVG2CMPMVS main_openMVG2CMPMVS.cpp)

target_link_libraries(openMVG_main_openMVG2CMPMVS
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_sfm
  stlplus
)

# - Export a SfM openMVG scene to CMPMVS format
#
add_executable(openMVG_main_openMVG2CMPMVS2 main_openMVG2CMPMVS2.cpp)

target_link_libraries(openMVG_main_openMVG2CMPMVS2
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_sfm
  stlplus
)


# - Export a SfM openMVG scene to MVE(v2) format
#
add_executable(openMVG_main_openMVG2MVE2 main_openMVG2MVE2.cpp)

target_link_libraries(openMVG_main_openMVG2MVE2
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_sfm
  stlplus
)

# - Export a SfM openMVG scene to meshlab scene with rasters
# -
add_executable(openMVG_main_openMVG2MESHLAB main_openMVG2MESHLAB.cpp)

target_link_libraries(openMVG_main_openMVG2MESHLAB
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_sfm
  stlplus
)

# - Export a SfM openMVG scene to mvs-texturing scene folder
# -
add_executable(openMVG_main_openMVG2MVSTEXTURING main_openMVG2MVSTEXTURING.cpp)

target_link_libraries(openMVG_main_openMVG2MVSTEXTURING
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_sfm
  stlplus
)

# - Export SfM openMVG camera scene as triangle meshes
# -
add_executable(openMVG_main_ExportCameraFrustums main_ExportCameraFrustums.cpp)

target_link_libraries(openMVG_main_ExportCameraFrustums
  openMVG_system
  openMVG_features
  openMVG_sfm
  stlplus
)

# installation rules
set_property(TARGET openMVG_main_openMVG2PMVS
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_openMVG2PMVS
  DESTINATION bin/
)

set_property(TARGET openMVG_main_openMVG2CMPMVS
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_openMVG2CMPMVS
  DESTINATION bin/
)

set_property(TARGET openMVG_main_openMVG2CMPMVS2
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_openMVG2CMPMVS2
  DESTINATION bin/
)

set_property(TARGET openMVG_main_openMVG2MESHLAB
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_openMVG2MESHLAB
  DESTINATION bin/
)

set_property(TARGET openMVG_main_openMVG2MVSTEXTURING
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_openMVG2MVSTEXTURING
  DESTINATION bin/
)

set_property(TARGET openMVG_main_openMVG2MVE2
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_openMVG2MVE2
  DESTINATION bin/
)

set_property(TARGET openMVG_main_ExportCameraFrustums
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_ExportCameraFrustums
  DESTINATION bin/
)

###
# SfM Research tools
#
# - Quality comparison against a GT camera path (MultiView Evaluation dataset)
###
add_executable(openMVG_main_evalQuality main_evalQuality.cpp)

target_link_libraries(openMVG_main_evalQuality
  openMVG_system
  openMVG_features
  openMVG_sfm
  stlplus
)

#installation rules
set_property(TARGET openMVG_main_evalQuality
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_evalQuality
  DESTINATION bin/
)


###
# SfM tools to prepare picture to a future SfM process
###

# - Export keyframes from video files / image sequence directories
# -
if(OPENMVG_HAVE_BOOST AND OPENMVG_HAVE_OIIO)

add_executable(openMVG_main_keyframeSelection main_keyframeSelection.cpp)
target_link_libraries(openMVG_main_keyframeSelection
  openMVG_system
  openMVG_keyframe
  ${OPENIMAGEIO_LIBRARIES}
  ${Boost_LIBRARIES}
)

#installation rules
set_property(TARGET openMVG_main_keyframeSelection
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_keyframeSelection
  DESTINATION bin/
)

endif()

###
# Support tool that converts an input video to a series of JPG or PNG files.
# Optionally writing XML files to add basic EXIF information.
###

if(OPENMVG_HAVE_LIBAV)

  add_executable(openMVG_main_convertVid2Img main_convertVid2Img.cpp)
  target_link_libraries(openMVG_main_convertVid2Img
    openMVG_image
    ${LIBAV_LIBRARIES}
    stlplus
  )

  target_include_directories(openMVG_main_convertVid2Img
    PRIVATE ${LIBAV_INCLUDE_DIRS}
  )

  #installation rules
  set_property(TARGET openMVG_main_convertVid2Img
    PROPERTY FOLDER OpenMVG/software
  )

  install(TARGETS openMVG_main_convertVid2Img
    DESTINATION bin/
  )

endif(OPENMVG_HAVE_LIBAV)

##
# Export tutorial with valid path thanks to a configuration file
##
set(OPENMVG_SOFTWARE_SFM_SRC_DIR ${CMAKE_CURRENT_LIST_DIR})
if(MSVC OR APPLE)
  set(OPENMVG_SOFTWARE_SFM_BUILD_DIR "${EXECUTABLE_OUTPUT_PATH}/${CMAKE_BUILD_TYPE}")
else()
  set(OPENMVG_SOFTWARE_SFM_BUILD_DIR ${EXECUTABLE_OUTPUT_PATH})
endif()

set(OPENMVG_TUTORIAL_IN_FILE "${CMAKE_CURRENT_LIST_DIR}/tutorial_demo.py.in")
configure_file(${OPENMVG_TUTORIAL_IN_FILE}
    "${CMAKE_CURRENT_BINARY_DIR}/tutorial_demo.py" @ONLY)

set(OPENMVG_TUTORIAL_IN_FILE "${CMAKE_CURRENT_LIST_DIR}/SfM_Pipeline.py.in")
configure_file(${OPENMVG_TUTORIAL_IN_FILE}
    "${CMAKE_CURRENT_BINARY_DIR}/SfM_Pipeline.py" @ONLY)
