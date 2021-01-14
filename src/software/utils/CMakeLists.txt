## AliceVision
## Utilities software

# Software PROPERTY FOLDER is 'Software/Utils'
set(FOLDER_SOFTWARE_UTILS "Software/Utils")


if(ALICEVISION_HAVE_CUDA)
alicevision_add_software(aliceVision_utils_hardwareResources
  SOURCE main_hardwareResources.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_system
        aliceVision_gpu
        Boost::program_options
)
endif()

if(ALICEVISION_BUILD_SFM)
# Uncertainty
if(ALICEVISION_HAVE_UNCERTAINTYTE)
    alicevision_add_software(aliceVision_utils_computeUncertainty
      SOURCE main_computeUncertainty.cpp
      FOLDER ${FOLDER_SOFTWARE_UTILS}
      LINKS aliceVision_sfm
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_cusparse_LIBRARY}
            ${UNCERTAINTYTE_LIBRARY}
            Boost::program_options
      INCLUDE_DIRS ${UNCERTAINTYTE_INCLUDE_DIR}
    )
# message(warning "UNCERTAINTYTE_LIBRARY: ${UNCERTAINTYTE_LIBRARY}")
# message(warning "CUDA_LIBRARIES: ${CUDA_LIBRARIES}")
# message(warning "CUDA_CUBLAS_LIBRARIES: ${CUDA_CUBLAS_LIBRARIES}")
# message(warning "CUDA_cusparse_LIBRARY: ${CUDA_cusparse_LIBRARY}")
endif()

alicevision_add_software(aliceVision_utils_imageProcessing 
    SOURCE main_imageProcessing.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS aliceVision_system
        aliceVision_image
        aliceVision_feature
        aliceVision_sfm
        aliceVision_sfmData
        aliceVision_sfmDataIO
        ${Boost_LIBRARIES}
)

if(ALICEVISION_HAVE_OPENCV)
  target_link_libraries(aliceVision_utils_imageProcessing PRIVATE ${OpenCV_LIBS})
endif()

# Voctree creation
alicevision_add_software(aliceVision_utils_voctreeCreation
  SOURCE main_voctreeCreation.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_voctree
        aliceVision_feature
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
)

# Voctree query utility
alicevision_add_software(aliceVision_utils_voctreeQueryUtility
  SOURCE main_voctreeQueryUtility.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_voctree
        aliceVision_sfm
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
        Boost::boost
        Boost::timer
)

# Voctree statistics
alicevision_add_software(aliceVision_utils_voctreeStatistics
  SOURCE main_voctreeStatistics.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_voctree
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
        Boost::boost
)

# Frustrum filtering
alicevision_add_software(aliceVision_utils_frustumFiltering
  SOURCE main_frustumFiltering.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_system
        aliceVision_feature
        aliceVision_matchingImageCollection
        aliceVision_sfm
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
        Boost::filesystem
)

# Transform rig
if(ALICEVISION_HAVE_ALEMBIC)
  alicevision_add_software(aliceVision_utils_rigTransform
    SOURCE main_rigTransform.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS aliceVision_sfmData
          aliceVision_sfmDataIO
          aliceVision_localization
          aliceVision_dataio
          aliceVision_rig
          Boost::program_options
          Boost::filesystem
  )
endif()

# SfM quality evaluation
# - quality comparison against a GT camera path (MultiView Evaluation dataset)
alicevision_add_software(aliceVision_utils_qualityEvaluation
  SOURCE main_qualityEvaluation.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_system
        aliceVision_feature
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
        Boost::filesystem
)

# SfM alignment
alicevision_add_software(aliceVision_utils_sfmAlignment
  SOURCE main_sfmAlignment.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_system
        aliceVision_feature
        aliceVision_sfm
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
)

# SfM transfer
alicevision_add_software(aliceVision_utils_sfmTransfer
  SOURCE main_sfmTransfer.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_system
        aliceVision_feature
        aliceVision_sfm
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
)

# SfM transform
alicevision_add_software(aliceVision_utils_sfmTransform
  SOURCE main_sfmTransform.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_system
        aliceVision_feature
        aliceVision_sfm
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
)

# SfM color harmonize
alicevision_add_software(aliceVision_utils_sfmColorHarmonize
  SOURCE main_sfmColorHarmonize.cpp
         sfmColorHarmonize/colorHarmonizeEngineGlobal.cpp # TODO
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_system
        aliceVision_image
        aliceVision_feature
        aliceVision_kvld
        aliceVision_sfm
        aliceVision_sfmData
        aliceVision_sfmDataIO
        aliceVision_colorHarmonization
        ${CLP_LIBRARIES}
        ${COINUTILS_LIBRARY}
        ${OSI_LIBRARY}
        Boost::program_options
        Boost::filesystem
        Boost::boost
        Boost::timer
)

# SfM localization
alicevision_add_software(aliceVision_utils_sfmLocalization
  SOURCE main_sfmLocalization.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_system
        aliceVision_feature
        aliceVision_sfm
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
        Boost::filesystem
)

# Keyframe selection
# - export keyframes from video files / image sequence directories
alicevision_add_software(aliceVision_utils_keyframeSelection
  SOURCE main_keyframeSelection.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_system
        aliceVision_keyframe
        ${OPENIMAGEIO_LIBRARIES}
        Boost::program_options
        Boost::filesystem
)


# Split 360 images in input in order to export square images
alicevision_add_software(aliceVision_utils_split360Images
  SOURCE main_split360Images.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_system
        aliceVision_numeric
        aliceVision_image
        aliceVision_panorama
        ${OPENIMAGEIO_LIBRARIES}
        Boost::program_options
        Boost::filesystem
)


# Project 180° fisheye images into equirectangular
alicevision_add_software(aliceVision_utils_fisheyeProjection
  SOURCE main_fisheyeProjection.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_system
        aliceVision_numeric
        aliceVision_image
        ${OPENIMAGEIO_LIBRARIES}
        ${Boost_LIBRARIES}
)

# Lighting estimation from picture, albedo and geometry
alicevision_add_software(aliceVision_utils_lightingEstimation
  SOURCE main_lightingEstimation.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_system
        aliceVision_numeric
        aliceVision_sfmData
        aliceVision_sfmDataIO
        aliceVision_mvsUtils
        aliceVision_image
        aliceVision_lightingEstimation
        ${Boost_LIBRARIES}
)


# Perform color chart detection
if(ALICEVISION_HAVE_OPENCV)
  alicevision_add_software(aliceVision_utils_colorCheckerDetection
    SOURCE main_colorCheckerDetection.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS aliceVision_system
          aliceVision_sfmData
          aliceVision_sfmDataIO
          Boost::program_options
          Boost::filesystem
          ${OpenCV_LIBS} # opencv_core  opencv_imgproc  opencv_video  opencv_imgcodecs  opencv_videoio  opencv_features2d  opencv_xfeatures2d
          opencv_mcc
)
endif()


# Perform color correction
if(ALICEVISION_HAVE_OPENCV)
  alicevision_add_software(aliceVision_utils_colorCheckerCorrection
    SOURCE main_colorCheckerCorrection.cpp
    FOLDER ${FOLDER_SOFTWARE_UTILS}
    LINKS aliceVision_system
          aliceVision_sfmData
          aliceVision_sfmDataIO
          Boost::program_options
          Boost::filesystem
          ${OpenCV_LIBS} # opencv_core  opencv_imgproc  opencv_video  opencv_imgcodecs  opencv_videoio  opencv_features2d  opencv_xfeatures2d
          opencv_mcc
)
endif()

endif() # ALICEVISION_BUILD_SFM

if(ALICEVISION_BUILD_MVS)

# Merge two meshes
alicevision_add_software(aliceVision_utils_mergeMeshes
  SOURCE main_mergeMeshes.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_system
        aliceVision_numeric
	Geogram::geogram
        ${Boost_LIBRARIES}
)

endif() # ALICEVISION_BUILD_MVS
