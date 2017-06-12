add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(openMVG_main_exifParsing exifParsing.cpp)

target_link_libraries(openMVG_main_exifParsing easyexif)

set_property(TARGET openMVG_main_exifParsing
  PROPERTY FOLDER OpenMVG/Samples
)
