add_executable(aliceVision_main_ParseDatabase main_ParseDatabase.cpp)

target_link_libraries(aliceVision_main_ParseDatabase stlplus)

set_property(TARGET aliceVision_main_ParseDatabase
  PROPERTY FOLDER AliceVision/Samples
)

