add_executable(openMVG_main_ParseDatabase main_ParseDatabase.cpp)

target_link_libraries(openMVG_main_ParseDatabase stlplus)

set_property(TARGET openMVG_main_ParseDatabase
  PROPERTY FOLDER AliceVision/Samples
)

