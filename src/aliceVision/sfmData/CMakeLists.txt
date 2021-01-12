# Headers
set(sfmData_files_headers
  SfMData.hpp
  CameraPose.hpp
  Landmark.hpp
  View.hpp
  Rig.hpp
  uid.hpp
  colorize.hpp
)

# Sources
set(sfmData_files_sources
  SfMData.cpp
  uid.cpp
  View.cpp
  colorize.cpp
)

alicevision_add_library(aliceVision_sfmData
  SOURCES ${sfmData_files_headers} ${sfmData_files_sources}
  PUBLIC_LINKS
    aliceVision_system
    aliceVision_numeric
    aliceVision_image
    aliceVision_feature
    aliceVision_geometry
    aliceVision_camera
    aliceVision_stl
    Boost::filesystem
  PRIVATE_LINKS
    Boost::boost
)

# Unit tests

alicevision_add_test(sfmData_test.cpp
  NAME "sfmData"
  LINKS aliceVision_sfmData
        aliceVision_system
)
alicevision_add_test(view_test.cpp
  NAME "view"
  LINKS aliceVision_sfmData
)