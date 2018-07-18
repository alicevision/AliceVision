## AliceVision
## Convertion software

# Convert SfMData format (from one to another)

add_executable(aliceVision_convertSfMFormat main_convertSfMFormat.cpp)

target_link_libraries(aliceVision_convertSfMFormat
	aliceVision_system
	aliceVision_feature
	aliceVision_sfm
	${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_convertSfMFormat
	PROPERTY FOLDER Software/Convert
)

install(TARGETS aliceVision_convertSfMFormat
	DESTINATION bin/
)

# Convert float descriptors to unsigned char

add_executable(aliceVision_convertFloatDescriptorToUchar main_convertFloatDescriptorToUchar.cpp)

target_link_libraries(aliceVision_convertFloatDescriptorToUchar
	aliceVision_localization
	aliceVision_feature
	${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_convertFloatDescriptorToUchar
	PROPERTY FOLDER Software/Convert
)

install(TARGETS aliceVision_convertFloatDescriptorToUchar
	DESTINATION bin/
)

# Convert image to EXR

add_executable(aliceVision_convertRAW main_convertRAW.cpp)

target_link_libraries(aliceVision_convertRAW
  aliceVision_localization
  aliceVision_feature
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_convertRAW
  PROPERTY FOLDER Software/Convert
)

install(TARGETS aliceVision_convertRAW
  DESTINATION bin/
)
