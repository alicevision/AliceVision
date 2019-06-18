## AliceVision
## Utilities software

# Software PROPERTY FOLDER is 'Software/Utils'
set(FOLDER_SOFTWARE_UTILS "Software/Utils")

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
            ${Boost_LIBRARIES}
      INCLUDE_DIRS ${UNCERTAINTYTE_INCLUDE_DIR}
    )
# message(warning "UNCERTAINTYTE_LIBRARY: ${UNCERTAINTYTE_LIBRARY}")
# message(warning "CUDA_LIBRARIES: ${CUDA_LIBRARIES}")
# message(warning "CUDA_CUBLAS_LIBRARIES: ${CUDA_CUBLAS_LIBRARIES}")
# message(warning "CUDA_cusparse_LIBRARY: ${CUDA_cusparse_LIBRARY}")
endif()

# Voctree creation
alicevision_add_software(aliceVision_utils_voctreeCreation
  SOURCE main_voctreeCreation.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_voctree
        aliceVision_feature
        aliceVision_sfmData
        aliceVision_sfmDataIO
        ${Boost_LIBRARIES}
)

# Voctree query utility
alicevision_add_software(aliceVision_utils_voctreeQueryUtility
  SOURCE main_voctreeQueryUtility.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_voctree
        aliceVision_sfm
        aliceVision_sfmData
        aliceVision_sfmDataIO
        ${Boost_LIBRARIES}
)

# Voctree statistics
alicevision_add_software(aliceVision_utils_voctreeStatistics
  SOURCE main_voctreeStatistics.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_voctree
        aliceVision_sfmData
        aliceVision_sfmDataIO
        ${Boost_LIBRARIES}
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
        ${Boost_LIBRARIES}
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
          ${Boost_LIBRARIES}
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
        ${Boost_LIBRARIES}
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
        ${Boost_LIBRARIES}
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
        ${Boost_LIBRARIES}
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
        ${Boost_LIBRARIES}
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
        ${Boost_LIBRARIES}
)

# Keyframe selection
# - export keyframes from video files / image sequence directories
alicevision_add_software(aliceVision_utils_keyframeSelection
  SOURCE main_keyframeSelection.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_system
        aliceVision_keyframe
        ${OPENIMAGEIO_LIBRARIES}
        ${Boost_LIBRARIES}
)


# Split 360 images in input in order to export square images
alicevision_add_software(aliceVision_utils_split360Images
  SOURCE main_split360Images.cpp
  FOLDER ${FOLDER_SOFTWARE_UTILS}
  LINKS aliceVision_system
        aliceVision_numeric
        aliceVision_image
        ${OPENIMAGEIO_LIBRARIES}
        ${Boost_LIBRARIES}
)

endif() # ALICEVISION_BUILD_SFM
