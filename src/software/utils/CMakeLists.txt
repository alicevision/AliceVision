## AliceVision
## Utilities software

# Voctree creation

add_executable(aliceVision_utils_voctreeCreation main_voctreeCreation.cpp)

target_link_libraries(aliceVision_utils_voctreeCreation
	aliceVision_voctree
	aliceVision_features
	${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_utils_voctreeCreation
  PROPERTY FOLDER AliceVision/Software/Utils
)

install(TARGETS aliceVision_utils_voctreeCreation
	DESTINATION bin/
)

# Voctree query utility

add_executable(aliceVision_utils_voctreeQueryUtility main_voctreeQueryUtility.cpp)

target_link_libraries(aliceVision_utils_voctreeQueryUtility
	aliceVision_voctree
	${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_utils_voctreeQueryUtility
  PROPERTY FOLDER AliceVision/Software/Utils
)

install(TARGETS aliceVision_utils_voctreeQueryUtility
	DESTINATION bin/
)

# Voctree statistics

add_executable(aliceVision_utils_voctreeStatistics main_voctreeStatistics.cpp)

target_link_libraries(aliceVision_utils_voctreeStatistics
	aliceVision_voctree
	${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_utils_voctreeStatistics
  PROPERTY FOLDER AliceVision/Software/Utils
)

install(TARGETS aliceVision_utils_voctreeStatistics
	DESTINATION bin/
)


# Frustrum filtering

add_executable(aliceVision_utils_frustumFiltering main_frustumFiltering.cpp)

target_link_libraries(aliceVision_utils_frustumFiltering
  aliceVision_system
  aliceVision_features
  aliceVision_sfm
  stlplus
)

set_property(TARGET aliceVision_utils_frustumFiltering
  PROPERTY FOLDER AliceVision/Software/Utils
)

install(TARGETS aliceVision_utils_frustumFiltering
  DESTINATION bin/
)

# Transform rig
  
if(ALICEVISION_HAVE_ALEMBIC)
	add_executable(aliceVision_utils_rigTransform main_rigTransform.cpp)

	target_link_libraries(aliceVision_utils_rigTransform
		aliceVision_sfm
		aliceVision_localization
		aliceVision_dataio
		aliceVision_rig
		${ABC_LIBRARIES}
		${BOOST_LIBRARIES}
	)
	
	set_property(TARGET aliceVision_utils_rigTransform
		PROPERTY FOLDER AliceVision/Software/Utils
	)

	install(TARGETS aliceVision_utils_rigTransform
		DESTINATION bin/
	)
endif()

# SfM quality evaluation
# - quality comparison against a GT camera path (MultiView Evaluation dataset)

add_executable(aliceVision_utils_qualityEvaluation main_qualityEvaluation.cpp)

target_link_libraries(aliceVision_utils_qualityEvaluation
  aliceVision_system
  aliceVision_features
  aliceVision_sfm
  stlplus
)

set_property(TARGET aliceVision_utils_qualityEvaluation
  PROPERTY FOLDER AliceVision/Software/Utils
)

install(TARGETS aliceVision_utils_qualityEvaluation
  DESTINATION bin/
)

# SfM alignment

add_executable(aliceVision_utils_sfmAlignment main_sfmAlignment.cpp)

target_link_libraries(aliceVision_utils_sfmAlignment
  aliceVision_system
  aliceVision_features
  aliceVision_sfm
  stlplus
)

set_property(TARGET aliceVision_utils_sfmAlignment
  PROPERTY FOLDER AliceVision/Software/Utils
)

install(TARGETS aliceVision_utils_sfmAlignment
  DESTINATION bin/
)

# SfM color harmonize

add_executable(aliceVision_utils_sfmColorHarmonize
  main_sfmColorHarmonize.cpp
  sfmColorHarmonize/colorHarmonizeEngineGlobal.cpp
)

target_link_libraries(aliceVision_utils_sfmColorHarmonize
  aliceVision_system
  aliceVision_image
  aliceVision_kvld
  aliceVision_features
  aliceVision_sfm
  stlplus
  ${CLP_LIBRARIES}
  ${COINUTILS_LIBRARY}
  ${OSI_LIBRARY}
)

set_property(TARGET aliceVision_utils_sfmColorHarmonize
  PROPERTY FOLDER AliceVision/Software/Utils
)

install(TARGETS aliceVision_utils_sfmColorHarmonize
  DESTINATION bin/
)

# Keyframe selection
# - export keyframes from video files / image sequence directories

if(ALICEVISION_HAVE_OIIO)
	add_executable(aliceVision_utils_keyframeSelection main_keyframeSelection.cpp)

	target_link_libraries(aliceVision_utils_keyframeSelection
	  aliceVision_system
	  aliceVision_keyframe
	  ${OPENIMAGEIO_LIBRARIES}
	  ${Boost_LIBRARIES}
	)

	set_property(TARGET aliceVision_utils_keyframeSelection
	  PROPERTY FOLDER AliceVision/Software/Utils
	)

	install(TARGETS aliceVision_utils_keyframeSelection
	  DESTINATION bin/
	)
endif()
