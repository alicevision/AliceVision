add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

project(exif)

# Headers
set(exif_files_headers
  exif_IO_EasyExif.hpp
  exif_IO.hpp
  sensor_width_database/datasheet.hpp
  sensor_width_database/ParseDatabase.hpp
)

# Sources
set(exif_files_sources
  exif_IO.cpp
)

set_source_files_properties(${exif_files_sources} PROPERTIES LANGUAGE CXX)

add_library(aliceVision_exif
  ${exif_files_headers}
  ${exif_files_sources}
)

target_link_libraries(aliceVision_exif
  stlplus
  easyexif
  ${LOG_LIB}
)

set_target_properties(aliceVision_exif
  PROPERTIES SOVERSION ${OPENMVG_VERSION_MAJOR}
  VERSION "${OPENMVG_VERSION_MAJOR}.${OPENMVG_VERSION_MINOR}"
)

install(TARGETS aliceVision_exif
  DESTINATION lib
  EXPORT aliceVision-targets
)

set_property(TARGET aliceVision_exif
  PROPERTY FOLDER AliceVision/AliceVision
)

UNIT_TEST(aliceVision exif_IO "aliceVision_exif")

add_subdirectory(sensor_width_database)
