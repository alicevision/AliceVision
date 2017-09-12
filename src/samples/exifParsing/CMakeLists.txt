add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_samples_exifParsing main_exifParsing.cpp)

target_link_libraries(aliceVision_samples_exifParsing easyexif)

set_property(TARGET aliceVision_samples_exifParsing
  PROPERTY FOLDER AliceVision/Samples
)
