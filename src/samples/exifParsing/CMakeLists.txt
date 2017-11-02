add_executable(aliceVision_samples_exifParsing main_exifParsing.cpp)

target_link_libraries(aliceVision_samples_exifParsing
  aliceVision_system
  easyexif
  ${BOOST_LIBRARIES}
)

set_property(TARGET aliceVision_samples_exifParsing
  PROPERTY FOLDER AliceVision/Samples
)
