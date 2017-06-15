if(OPENMVG_HAVE_BOOST)

  ###
  # Test the localization class
  ###
  add_executable(openMVG_main_cameraLocalizer main_cameraLocalizer.cpp)
  install(TARGETS openMVG_main_cameraLocalizer DESTINATION bin/)
  target_link_libraries(openMVG_main_cameraLocalizer
    openMVG_localization
    openMVG_dataio
    openMVG_image
    openMVG_features
    vlsift
    ${BOOST_LIBRARIES}
  )

  if(OPENMVG_HAVE_ALEMBIC)
      target_link_libraries(openMVG_main_cameraLocalizer openMVG_sfm ${ABC_LIBRARIES})
  endif()

  if(OPENMVG_HAVE_CCTAG)
      target_link_libraries(openMVG_main_cameraLocalizer CCTag::CCTag)
  endif()

  ###
  # Create the vocabulary tree
  ###
  add_executable(openMVG_main_createVoctree createVoctree.cpp)

  install(TARGETS openMVG_main_createVoctree
    DESTINATION bin/
  )

  target_link_libraries(openMVG_main_createVoctree
    openMVG_voctree
    openMVG_features
    ${Boost_LIBRARIES}
  )

  ###
  # Generate the pair lists
  ###
  add_executable(openMVG_main_generatePairList pairListFromVoctree.cpp)

  install(TARGETS openMVG_main_generatePairList
    DESTINATION bin/
  )

  target_link_libraries(openMVG_main_generatePairList
    openMVG_voctree
    ${Boost_LIBRARIES}
  )

  ###
  # Voctree query utility
  ###
  add_executable(openMVG_main_voctreeQueryUtility voctreeQueryUtility.cpp)

  install(TARGETS openMVG_main_voctreeQueryUtility
    DESTINATION bin/
  )

  target_link_libraries(openMVG_main_voctreeQueryUtility
    openMVG_voctree
    ${Boost_LIBRARIES}
  )

  ###
  # Test the localization class for the rig
  ###
  add_executable(openMVG_main_rigLocalizer main_rigLocalizer.cpp)

  install(TARGETS openMVG_main_rigLocalizer
    DESTINATION bin/
  )

  target_link_libraries(openMVG_main_rigLocalizer
    openMVG_localization
    openMVG_dataio
    openMVG_rig
    openMVG_image
    openMVG_features
    vlsift
    ${Boost_LIBRARIES}
  )

  if(OPENMVG_HAVE_ALEMBIC)
      target_link_libraries(openMVG_main_rigLocalizer openMVG_sfm ${ABC_LIBRARIES})
  endif()

  if(OPENMVG_HAVE_CCTAG)
      target_link_libraries(openMVG_main_rigLocalizer CCTag::CCTag)
  endif()

  if(OPENMVG_HAVE_ALEMBIC)

    add_executable(openMVG_main_rigTransform main_rigTransform.cpp)

    install(TARGETS openMVG_main_rigTransform
      DESTINATION bin/
    )

    target_link_libraries(openMVG_main_rigTransform
      openMVG_localization
      openMVG_dataio
      openMVG_rig
      ${BOOST_LIBRARIES}
    )

    target_link_libraries(openMVG_main_rigTransform openMVG_sfm ${ABC_LIBRARIES})

  endif()

  ###
  # utility to convert float descriptors to char
  ###
  add_executable(openMVG_main_convertDescFloat2Uchar convertDescFloat2Uchar.cpp)

  install(TARGETS openMVG_main_convertDescFloat2Uchar
    DESTINATION bin/
  )

  target_link_libraries(openMVG_main_convertDescFloat2Uchar
    openMVG_localization openMVG_features
    ${Boost_LIBRARIES}
  )

  # voctree statistics
  ###
  add_executable(openMVG_main_voctreeStatistics voctreeStatistics.cpp)

  install(TARGETS openMVG_main_voctreeStatistics
    DESTINATION bin/
  )

  target_link_libraries(openMVG_main_voctreeStatistics
    openMVG_voctree
    ${Boost_LIBRARIES}
  )

  #@TODO decide where to put the binaries

  # Installation rules
  #set_property(TARGET openMVG_main_createVoctree PROPERTY FOLDER OpenMVG/software)
  #install(TARGETS openMVG_main_createVoctree DESTINATION bin/)
  #set_property(TARGET openMVG_main_generatePairList PROPERTY FOLDER OpenMVG/software)
  #install(TARGETS openMVG_main_generatePairList DESTINATION bin/)

endif()



###
# Localize images in an existing SfM reconstruction
###
add_executable(openMVG_main_SfM_Localization main_SfM_Localization.cpp)

target_link_libraries(openMVG_main_SfM_Localization
  openMVG_system
  openMVG_image
  openMVG_features
  openMVG_matching_image_collection
  openMVG_sfm
  easyexif
  vlsift
)

# Installation rules
set_property(TARGET openMVG_main_SfM_Localization
  PROPERTY FOLDER OpenMVG/software
)

install(TARGETS openMVG_main_SfM_Localization
  DESTINATION bin/
)
