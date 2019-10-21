## AliceVision
## Pipeline softwares

# Software PROPERTY FOLDER is 'Software/Pipeline'
set(FOLDER_SOFTWARE_PIPELINE "Software/Pipeline")

### SfM software
if(ALICEVISION_BUILD_SFM)

  # Intrinsic image analysis and SfMData container initialization
  alicevision_add_software(aliceVision_cameraInit
    SOURCE main_cameraInit.cpp
    FOLDER ${FOLDER_SOFTWARE_PIPELINE}
    LINKS aliceVision_system
          aliceVision_sensorDB
          aliceVision_image
          aliceVision_feature
          aliceVision_sfmData
          aliceVision_sfmDataIO
          ${Boost_LIBRARIES}
  )

  # Image processing
  if(ALICEVISION_HAVE_OPENCV)
    alicevision_add_software(aliceVision_imageMasking
      SOURCE main_imageMasking.cpp
      FOLDER ${FOLDER_SOFTWARE_PIPELINE}
      LINKS aliceVision_image
            aliceVision_imageMasking
            aliceVision_system
            aliceVision_sfmData
            aliceVision_sfmDataIO
            ${OpenCV_LIBRARIES}
            ${Boost_LIBRARIES}
    )
  endif()

  # Feature extraction
  alicevision_add_software(aliceVision_featureExtraction
    SOURCE main_featureExtraction.cpp
    FOLDER ${FOLDER_SOFTWARE_PIPELINE}
    LINKS aliceVision_system
          aliceVision_gpu
          aliceVision_image
          aliceVision_feature
          aliceVision_multiview
          aliceVision_sfmData
          aliceVision_sfmDataIO
          vlsift
          ${Boost_LIBRARIES}
  )

  # Image matching
  # - generate the image pair lists
  alicevision_add_software(aliceVision_imageMatching
    SOURCE main_imageMatching.cpp
    FOLDER ${FOLDER_SOFTWARE_PIPELINE}
    LINKS aliceVision_system
          aliceVision_sfmData
          aliceVision_sfmDataIO
          aliceVision_voctree
          ${Boost_LIBRARIES}
  )

  # Feature matching
  # - putative matches and geometric filtered matches
  alicevision_add_software(aliceVision_featureMatching
    SOURCE main_featureMatching.cpp
    FOLDER ${FOLDER_SOFTWARE_PIPELINE}
    LINKS aliceVision_system
          aliceVision_feature
          aliceVision_multiview
          aliceVision_matchingImageCollection
          aliceVision_sfm
          aliceVision_sfmData
          aliceVision_sfmDataIO
          ${Boost_LIBRARIES}
  )

  # Incremental / Sequential SfM
  alicevision_add_software(aliceVision_incrementalSfM
    SOURCE main_incrementalSfM.cpp
    FOLDER ${FOLDER_SOFTWARE_PIPELINE}
    LINKS aliceVision_system
          aliceVision_image
          aliceVision_feature
          aliceVision_sfm
          aliceVision_sfmData
          aliceVision_sfmDataIO
          ${Boost_LIBRARIES}
  )

  # Global SfM
  alicevision_add_software(aliceVision_globalSfM
    SOURCE main_globalSfM.cpp
    FOLDER ${FOLDER_SOFTWARE_PIPELINE}
    LINKS aliceVision_system
          aliceVision_image
          aliceVision_feature
          aliceVision_sfm
          aliceVision_sfmData
          aliceVision_sfmDataIO
          ${Boost_LIBRARIES}
  )

  # Compute structure from known camera poses
  alicevision_add_software(aliceVision_computeStructureFromKnownPoses
    SOURCE main_computeStructureFromKnownPoses.cpp
    FOLDER ${FOLDER_SOFTWARE_PIPELINE}
    LINKS aliceVision_system
          aliceVision_feature
          aliceVision_sfm
          aliceVision_sfmData
          aliceVision_sfmDataIO
          ${Boost_LIBRARIES}
  )

  # Calibrate a camera
  if(ALICEVISION_HAVE_OPENCV)
    alicevision_add_software(aliceVision_cameraCalibration
      SOURCE main_cameraCalibration.cpp
      FOLDER ${FOLDER_SOFTWARE_PIPELINE}
      LINKS aliceVision_dataio
            aliceVision_image
            aliceVision_calibration
            aliceVision_system
            ${OpenCV_LIBRARIES}
            ${Boost_LIBRARIES}
    )
  endif()

  # Calibrate a rig
  alicevision_add_software(aliceVision_rigCalibration
    SOURCE main_rigCalibration.cpp
    FOLDER ${FOLDER_SOFTWARE_PIPELINE}
    LINKS aliceVision_rig
          aliceVision_localization
          aliceVision_dataio
          aliceVision_image
          aliceVision_feature
          vlsift # TODO: delete vlsift
          ${Boost_LIBRARIES}
  )

  if(ALICEVISION_HAVE_CCTAG)
    target_link_libraries(aliceVision_rigCalibration PUBLIC CCTag::CCTag)
  endif()

  # Localize a camera
  alicevision_add_software(aliceVision_cameraLocalization
    SOURCE main_cameraLocalization.cpp
    FOLDER ${FOLDER_SOFTWARE_PIPELINE}
    LINKS aliceVision_localization
          aliceVision_dataio
          aliceVision_image
          aliceVision_feature
          vlsift
          ${Boost_LIBRARIES}
  )

  if(ALICEVISION_HAVE_CCTAG)
    target_link_libraries(aliceVision_cameraLocalization PUBLIC CCTag::CCTag)
  endif()

  # Localize a rig
  alicevision_add_software(aliceVision_rigLocalization
    SOURCE main_rigLocalization.cpp
    FOLDER ${FOLDER_SOFTWARE_PIPELINE}
    LINKS aliceVision_localization
          aliceVision_dataio
          aliceVision_rig
          aliceVision_image
          aliceVision_feature
          vlsift
          ${Boost_LIBRARIES}
  )

  if(ALICEVISION_HAVE_CCTAG)
    target_link_libraries(aliceVision_rigLocalization PUBLIC CCTag::CCTag)
  endif()

  # Prepare dense scene for MVS
  alicevision_add_software(aliceVision_prepareDenseScene
    SOURCE main_prepareDenseScene.cpp
    FOLDER ${FOLDER_SOFTWARE_PIPELINE}
    LINKS aliceVision_system
          aliceVision_image
          aliceVision_feature
          aliceVision_sfmData
          aliceVision_sfmDataIO
          ${Boost_LIBRARIES}
  )
endif() # if(ALICEVISION_BUILD_SFM)

### MVS software
if(ALICEVISION_BUILD_MVS)

  if(ALICEVISION_HAVE_CUDA) # Depth map computation need CUDA
    # Depth Map Estimation
    alicevision_add_software(aliceVision_depthMapEstimation
      SOURCE main_depthMapEstimation.cpp
      FOLDER ${FOLDER_SOFTWARE_PIPELINE}
      LINKS aliceVision_system
            aliceVision_gpu
            aliceVision_mvsData
            aliceVision_mvsUtils
            aliceVision_depthMap
            aliceVision_sfmData
            aliceVision_sfmDataIO
            ${Boost_LIBRARIES}
    )

    # Depth Map Filtering
    alicevision_add_software(aliceVision_depthMapFiltering
      SOURCE main_depthMapFiltering.cpp
      FOLDER ${FOLDER_SOFTWARE_PIPELINE}
      LINKS aliceVision_system
            aliceVision_mvsData
            aliceVision_mvsUtils
            aliceVision_fuseCut
            aliceVision_depthMap
            aliceVision_sfmData
            aliceVision_sfmDataIO
            ${Boost_LIBRARIES}
    )
  endif()

  # Meshing
  alicevision_add_software(aliceVision_meshing
    SOURCE main_meshing.cpp
    FOLDER ${FOLDER_SOFTWARE_PIPELINE}
    LINKS aliceVision_system
          aliceVision_mvsData
          aliceVision_mvsUtils
          aliceVision_mesh
          aliceVision_fuseCut
          aliceVision_sfmData
          aliceVision_sfmDataIO
          ${Boost_LIBRARIES}
  )

  if(ALICEVISION_HAVE_MESHSDFILTER)

    # Mesh Denoising
    alicevision_add_software(aliceVision_meshDenoising
      SOURCE main_meshDenoising.cpp
      FOLDER ${FOLDER_SOFTWARE_PIPELINE}
      LINKS aliceVision_system
            aliceVision_mvsUtils
            MeshSDLibrary
            Eigen3::Eigen
            ${Boost_LIBRARIES}
    )

    # Mesh Decimate
    alicevision_add_software(aliceVision_meshDecimate
      SOURCE main_meshDecimate.cpp
      FOLDER ${FOLDER_SOFTWARE_PIPELINE}
      LINKS aliceVision_system
            aliceVision_mvsUtils
            OpenMesh
            ${Boost_LIBRARIES}
    )
  endif()

  # Mesh Filtering
  alicevision_add_software(aliceVision_meshFiltering
    SOURCE main_meshFiltering.cpp
    FOLDER ${FOLDER_SOFTWARE_PIPELINE}
    LINKS aliceVision_system
          aliceVision_mvsUtils
          aliceVision_mesh
          ${Boost_LIBRARIES}
  )

  # Mesh Resampling
  alicevision_add_software(aliceVision_meshResampling
    SOURCE main_meshResampling.cpp
    FOLDER ${FOLDER_SOFTWARE_PIPELINE}
    LINKS aliceVision_system
          aliceVision_mvsUtils
          Geogram::geogram
          ${Boost_LIBRARIES}
  )

  # Texturing
  alicevision_add_software(aliceVision_texturing
    SOURCE main_texturing.cpp
    FOLDER ${FOLDER_SOFTWARE_PIPELINE}
    LINKS aliceVision_system
          aliceVision_mvsData
          aliceVision_mvsUtils
          aliceVision_mesh
          aliceVision_sfmData
          aliceVision_sfmDataIO
          ${Boost_LIBRARIES}
  )
endif() # if(ALICEVISION_BUILD_MVS)
