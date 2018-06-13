add_executable(aliceVision_samples_parseDatabase main_parseDatabase.cpp)

target_link_libraries(aliceVision_samples_parseDatabase
  aliceVision_system
  aliceVision_sensorDB
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_samples_parseDatabase
  PROPERTY FOLDER Samples
)

