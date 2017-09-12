add_executable(aliceVision_samples_ParseDatabase main_parseDatabase.cpp)

target_link_libraries(aliceVision_samples_ParseDatabase stlplus)

set_property(TARGET aliceVision_samples_ParseDatabase
  PROPERTY FOLDER AliceVision/Samples
)

