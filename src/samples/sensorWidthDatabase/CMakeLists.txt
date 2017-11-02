add_executable(aliceVision_samples_parseDatabase main_parseDatabase.cpp)

target_link_libraries(aliceVision_samples_parseDatabase
  aliceVision_system
  stlplus
  ${BOOST_LIBRARIES}
)

set_property(TARGET aliceVision_samples_parseDatabase
  PROPERTY FOLDER AliceVision/Samples
)

