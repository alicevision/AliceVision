## AliceVision
## Exportation software

# Software PROPERTY FOLDER is 'Software/Export'
set(FOLDER_SOFTWARE_EXPORT "Software/Export")

if(ALICEVISION_BUILD_SFM)
# Export an alembic animated camera from a SfMData file
if(ALICEVISION_HAVE_ALEMBIC)
  alicevision_add_software(aliceVision_exportAnimatedCamera
    SOURCE main_exportAnimatedCamera.cpp
    FOLDER ${FOLDER_SOFTWARE_EXPORT}
    LINKS aliceVision_system
          aliceVision_sfmData
          aliceVision_sfmDataIO
          Boost::program_options
          Boost::filesystem
          Boost::boost
          Boost::timer
  )
endif()

# Export view extracted keypoints
alicevision_add_software(aliceVision_exportKeypoints
  SOURCE main_exportKeypoints.cpp
  FOLDER ${FOLDER_SOFTWARE_EXPORT}
  LINKS aliceVision_system
        aliceVision_feature
        aliceVision_sfm
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
        Boost::filesystem
        Boost::boost
        Boost::timer
)

# Export view computed matches (putatives, geometrics) per image pair
alicevision_add_software(aliceVision_exportMatches
  SOURCE main_exportMatches.cpp
  FOLDER ${FOLDER_SOFTWARE_EXPORT}
  LINKS aliceVision_system
        aliceVision_feature
        aliceVision_sfm
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
        Boost::filesystem
        Boost::boost
        Boost::timer
)

# Export view tracks per image pair
alicevision_add_software(aliceVision_exportTracks
  SOURCE main_exportTracks.cpp
  FOLDER ${FOLDER_SOFTWARE_EXPORT}
  LINKS aliceVision_system
        aliceVision_feature
        aliceVision_sfm
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
        Boost::filesystem
        Boost::boost
        Boost::timer
)

# Export a SfM aliceVision scene to PMVS format
alicevision_add_software(aliceVision_exportPMVS
  SOURCE main_exportPMVS.cpp
  FOLDER ${FOLDER_SOFTWARE_EXPORT}
  LINKS aliceVision_system
        aliceVision_image
        aliceVision_feature
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
        Boost::filesystem
        Boost::boost
        Boost::timer
)

# Export point cloud
alicevision_add_software(aliceVision_exportColoredPointCloud
  SOURCE main_exportColoredPointCloud.cpp
  FOLDER ${FOLDER_SOFTWARE_EXPORT}
  LINKS aliceVision_system
        aliceVision_image
        aliceVision_feature
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
)

# Export a SfM aliceVision scene to MayaMVG
if(ALICEVISION_HAVE_ALEMBIC) # maya can read alembic file
  alicevision_add_software(aliceVision_exportMeshroomMaya
    SOURCE main_exportMeshroomMaya.cpp
    FOLDER ${FOLDER_SOFTWARE_EXPORT}
    LINKS aliceVision_system
          aliceVision_image
          aliceVision_feature
          aliceVision_sfmData
          aliceVision_sfmDataIO
          Boost::program_options
          Boost::filesystem
          Boost::boost
          Boost::timer
          ${OPENIMAGEIO_LIBRARIES}
  )
endif()

# Export a SfM aliceVision scene to MVE(v2) format
alicevision_add_software(aliceVision_exportMVE2
  SOURCE main_exportMVE2.cpp
  FOLDER ${FOLDER_SOFTWARE_EXPORT}
  LINKS aliceVision_system
        aliceVision_image
        aliceVision_feature
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
        Boost::filesystem
        Boost::boost
        Boost::timer
)

# Export a SfM aliceVision scene to meshlab scene with rasters
alicevision_add_software(aliceVision_exportMeshlab
  SOURCE main_exportMeshlab.cpp
  FOLDER ${FOLDER_SOFTWARE_EXPORT}
  LINKS aliceVision_system
        aliceVision_image
        aliceVision_feature
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
        Boost::filesystem
        Boost::boost
        Boost::timer
)

# Export a SfM aliceVision scene to mvs-texturing scene folder
alicevision_add_software(aliceVision_exportMVSTexturing
  SOURCE main_exportMVSTexturing.cpp
  FOLDER ${FOLDER_SOFTWARE_EXPORT}
  LINKS aliceVision_system
        aliceVision_image
        aliceVision_feature
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
        Boost::filesystem
)

# Export a SfM aliceVision scene to Matlab
alicevision_add_software(aliceVision_exportMatlab
  SOURCE main_exportMatlab.cpp
  FOLDER ${FOLDER_SOFTWARE_EXPORT}
  LINKS aliceVision_system
        aliceVision_image
        aliceVision_feature
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
        Boost::filesystem
        Boost::boost
        Boost::timer
)

# Export SfM aliceVision camera scene as triangle meshes
alicevision_add_software(aliceVision_exportCameraFrustums
  SOURCE main_exportCameraFrustums.cpp
  FOLDER ${FOLDER_SOFTWARE_EXPORT}
  LINKS aliceVision_system
        aliceVision_feature
        aliceVision_sfm
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
        Boost::filesystem
)
endif()
