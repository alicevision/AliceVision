if(OPENMVG_HAVE_BOOST)

  ###
  # Test the localization class
  ###
  add_executable(aliceVision_main_cameraLocalizer main_cameraLocalizer.cpp)
  install(TARGETS aliceVision_main_cameraLocalizer DESTINATION bin/)
  target_link_libraries(aliceVision_main_cameraLocalizer
    aliceVision_localization
    aliceVision_dataio
    aliceVision_image
    aliceVision_features
    vlsift
    ${BOOST_LIBRARIES}
  )

  if(OPENMVG_HAVE_ALEMBIC)
      target_link_libraries(aliceVision_main_cameraLocalizer aliceVision_sfm ${ABC_LIBRARIES})
  endif()

  if(OPENMVG_HAVE_CCTAG)
      target_link_libraries(aliceVision_main_cameraLocalizer CCTag::CCTag)
  endif()

  ###
  # Create the vocabulary tree
  ###
  add_executable(aliceVision_main_createVoctree createVoctree.cpp)

  install(TARGETS aliceVision_main_createVoctree
    DESTINATION bin/
  )

  target_link_libraries(aliceVision_main_createVoctree
    aliceVision_voctree
    aliceVision_features
    ${Boost_LIBRARIES}
  )

  ###
  # Generate the pair lists
  ###
  add_executable(aliceVision_main_generatePairList pairListFromVoctree.cpp)

  install(TARGETS aliceVision_main_generatePairList
    DESTINATION bin/
  )

  target_link_libraries(aliceVision_main_generatePairList
    aliceVision_voctree
    ${Boost_LIBRARIES}
  )

  ###
  # Voctree query utility
  ###
  add_executable(aliceVision_main_voctreeQueryUtility voctreeQueryUtility.cpp)

  install(TARGETS aliceVision_main_voctreeQueryUtility
    DESTINATION bin/
  )

  target_link_libraries(aliceVision_main_voctreeQueryUtility
    aliceVision_voctree
    ${Boost_LIBRARIES}
  )

  ###
  # Test the localization class for the rig
  ###
  add_executable(aliceVision_main_rigLocalizer main_rigLocalizer.cpp)

  install(TARGETS aliceVision_main_rigLocalizer
    DESTINATION bin/
  )

  target_link_libraries(aliceVision_main_rigLocalizer
    aliceVision_localization
    aliceVision_dataio
    aliceVision_rig
    aliceVision_image
    aliceVision_features
    vlsift
    ${Boost_LIBRARIES}
  )

  if(OPENMVG_HAVE_ALEMBIC)
      target_link_libraries(aliceVision_main_rigLocalizer aliceVision_sfm ${ABC_LIBRARIES})
  endif()

  if(OPENMVG_HAVE_CCTAG)
      target_link_libraries(aliceVision_main_rigLocalizer CCTag::CCTag)
  endif()

  if(OPENMVG_HAVE_ALEMBIC)

    add_executable(aliceVision_main_rigTransform main_rigTransform.cpp)

    install(TARGETS aliceVision_main_rigTransform
      DESTINATION bin/
    )

    target_link_libraries(aliceVision_main_rigTransform
      aliceVision_localization
      aliceVision_dataio
      aliceVision_rig
      ${BOOST_LIBRARIES}
    )

    target_link_libraries(aliceVision_main_rigTransform aliceVision_sfm ${ABC_LIBRARIES})

  endif()

  ###
  # utility to convert float descriptors to char
  ###
  add_executable(aliceVision_main_convertDescFloat2Uchar convertDescFloat2Uchar.cpp)

  install(TARGETS aliceVision_main_convertDescFloat2Uchar
    DESTINATION bin/
  )

  target_link_libraries(aliceVision_main_convertDescFloat2Uchar
    aliceVision_localization
    aliceVision_features
    ${Boost_LIBRARIES}
  )

  # voctree statistics
  ###
  add_executable(aliceVision_main_voctreeStatistics voctreeStatistics.cpp)

  install(TARGETS aliceVision_main_voctreeStatistics
    DESTINATION bin/
  )

  target_link_libraries(aliceVision_main_voctreeStatistics
    aliceVision_voctree
    ${Boost_LIBRARIES}
  )

  #@TODO decide where to put the binaries

  # Installation rules
  #set_property(TARGET aliceVision_main_createVoctree PROPERTY FOLDER AliceVision/software)
  #install(TARGETS aliceVision_main_createVoctree DESTINATION bin/)
  #set_property(TARGET aliceVision_main_generatePairList PROPERTY FOLDER AliceVision/software)
  #install(TARGETS aliceVision_main_generatePairList DESTINATION bin/)

endif()



###
# Localize images in an existing SfM reconstruction
###
add_executable(aliceVision_main_SfM_Localization main_SfM_Localization.cpp)

target_link_libraries(aliceVision_main_SfM_Localization
  aliceVision_system
  aliceVision_image
  aliceVision_features
  aliceVision_matching_image_collection
  aliceVision_sfm
  easyexif
  vlsift
)

# Installation rules
set_property(TARGET aliceVision_main_SfM_Localization
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_SfM_Localization
  DESTINATION bin/
)
