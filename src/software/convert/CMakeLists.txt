## AliceVision
## Convertion software

# Software PROPERTY FOLDER is 'Software/Convert'
set(FOLDER_SOFTWARE_CONVERT "Software/Convert")

# Convert SfMData format (from one to another)
alicevision_add_software(aliceVision_convertSfMFormat
  SOURCE main_convertSfMFormat.cpp
  FOLDER ${FOLDER_SOFTWARE_CONVERT}
  LINKS aliceVision_system
        aliceVision_feature
        aliceVision_sfm
        ${Boost_LIBRARIES}
)

# Convert float descriptors to unsigned char
alicevision_add_software(aliceVision_convertFloatDescriptorToUchar
  SOURCE main_convertFloatDescriptorToUchar.cpp
  FOLDER ${FOLDER_SOFTWARE_CONVERT}
  LINKS aliceVision_localization
        aliceVision_feature
        ${Boost_LIBRARIES}
)

# Convert image to EXR
alicevision_add_software(aliceVision_convertRAW
  SOURCE main_convertRAW.cpp
  FOLDER ${FOLDER_SOFTWARE_CONVERT}
  LINKS aliceVision_system
        aliceVision_image
        ${Boost_LIBRARIES}
)
