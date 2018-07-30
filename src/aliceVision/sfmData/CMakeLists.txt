# Headers
set(sfmData_files_headers
  SfMData.hpp
  CameraPose.hpp
  Landmark.hpp
  View.hpp
  Rig.hpp
  uid.hpp
)

# Sources
set(sfmData_files_sources
  SfMData.cpp
  uid.cpp
)

alicevision_add_library(aliceVision_sfmData
  SOURCES ${sfmData_files_headers} ${sfmData_files_sources}
  PUBLIC_LINKS
    aliceVision_system
    aliceVision_numeric
    aliceVision_image
    aliceVision_geometry
    aliceVision_camera
    aliceVision_stl
    ${Boost_FILESYSTEM_LIBRARY}
)

# Unit tests

alicevision_add_test(rig_test.cpp
  NAME "sfmData_rig"
  LINKS aliceVision_sfmData
        aliceVision_feature
        aliceVision_system
)

