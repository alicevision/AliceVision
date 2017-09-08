###
# ColorHarmonize
###
add_executable(aliceVision_main_ColHarmonize
  main_ColHarmonize.cpp
  colorHarmonizeEngineGlobal.cpp
)

target_link_libraries(aliceVision_main_ColHarmonize
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

set_property(TARGET aliceVision_main_ColHarmonize
  PROPERTY FOLDER AliceVision/software
)

install(TARGETS aliceVision_main_ColHarmonize
  DESTINATION bin/
)
