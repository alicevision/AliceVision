## AliceVision
## Convertion software

# Software PROPERTY FOLDER is 'Software/Convert'
set(FOLDER_SOFTWARE_CONVERT "Software/Convert")

if(ALICEVISION_BUILD_SFM)
# Convert SfMData format (from one to another)
alicevision_add_software(aliceVision_convertSfMFormat
  SOURCE main_convertSfMFormat.cpp
  FOLDER ${FOLDER_SOFTWARE_CONVERT}
  LINKS aliceVision_system
        aliceVision_feature
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
        Boost::filesystem
        Boost::system
)

# Convert float descriptors to unsigned char
alicevision_add_software(aliceVision_convertFloatDescriptorToUchar
  SOURCE main_convertFloatDescriptorToUchar.cpp
  FOLDER ${FOLDER_SOFTWARE_CONVERT}
  LINKS aliceVision_localization
        aliceVision_feature
        Boost::program_options
        Boost::filesystem
        Boost::boost
        Boost::timer
)

alicevision_add_software(aliceVision_importKnownPoses
  SOURCE main_importKnownPoses.cpp
  FOLDER ${FOLDER_SOFTWARE_CONVERT}
  LINKS aliceVision_localization
        aliceVision_feature
        aliceVision_sfmData
        aliceVision_sfmDataIO
        Boost::program_options
        Boost::filesystem
        Boost::boost
        Boost::timer
)
endif() # ALICEVISION_BUILD_SFM

# Convert image to EXR
alicevision_add_software(aliceVision_convertRAW
  SOURCE main_convertRAW.cpp
  FOLDER ${FOLDER_SOFTWARE_CONVERT}
  LINKS aliceVision_system
        aliceVision_image
        Boost::program_options
        Boost::filesystem
)

if(ALICEVISION_BUILD_MVS)

# Merge two meshes
alicevision_add_software(aliceVision_convertMesh
  SOURCE main_convertMesh.cpp
  FOLDER ${FOLDER_SOFTWARE_CONVERT}
  LINKS aliceVision_system
        aliceVision_numeric
	aliceVision_mesh
        ${Boost_LIBRARIES}
)

endif() # ALICEVISION_BUILD_MVS