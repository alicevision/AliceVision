## AliceVision
## Convertion software

# Convert SfMData format (from one to another)

add_executable(aliceVision_convertSfMFormat main_convertSfMFormat.cpp)

target_link_libraries(aliceVision_convertSfMFormat
	aliceVision_system
	aliceVision_feature
	aliceVision_sfm
	stlplus
	${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_convertSfMFormat
	PROPERTY FOLDER AliceVision/Software/Convert
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
	PROPERTY FOLDER AliceVision/Software/Convert
)

install(TARGETS aliceVision_convertFloatDescriptorToUchar
	DESTINATION bin/
)

  
# Convert to an alembic animated camera

if(ALICEVISION_HAVE_ALEMBIC)
	add_executable(aliceVision_convertAnimatedCamera main_convertAnimatedCamera.cpp)

	target_link_libraries(aliceVision_convertAnimatedCamera
		aliceVision_system
		aliceVision_feature
		aliceVision_sfm
		stlplus
    ${BOOST_LIBRARIES}
	)

	set_property(TARGET aliceVision_convertAnimatedCamera
		PROPERTY FOLDER AliceVision/Software/Convert
	)

	install(TARGETS aliceVision_convertAnimatedCamera
		DESTINATION bin/
	)
endif()

# Convert uid

add_executable(aliceVision_convertUID main_convertUID.cpp)

target_link_libraries(aliceVision_convertUID
  aliceVision_localization
  aliceVision_feature
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_convertUID
  PROPERTY FOLDER AliceVision/Software/Convert
)

install(TARGETS aliceVision_convertUID
  DESTINATION bin/
)

# Convert image to EXR

add_executable(aliceVision_convertImageToEXR main_convertImageToEXR.cpp)

target_link_libraries(aliceVision_convertImageToEXR
  aliceVision_localization
  aliceVision_feature
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_convertImageToEXR
  PROPERTY FOLDER AliceVision/Software/Convert
)

install(TARGETS aliceVision_convertImageToEXR
  DESTINATION bin/
)
