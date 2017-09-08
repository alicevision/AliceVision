add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_main_exifParsing exifParsing.cpp)

target_link_libraries(aliceVision_main_exifParsing easyexif)

set_property(TARGET aliceVision_main_exifParsing
  PROPERTY FOLDER AliceVision/Samples
)
