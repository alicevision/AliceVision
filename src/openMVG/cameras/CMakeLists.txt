# Headers
set(cameras_files_headers
  Camera_Common.hpp
  Camera_Intrinsics.hpp
  Camera_Pinhole.hpp
  Camera_Pinhole_Brown.hpp
  Camera_Pinhole_Fisheye.hpp
  Camera_Pinhole_Fisheye1.hpp
  Camera_Pinhole_Radial.hpp
  cameras.hpp
  Camera_undistort_image.hpp
)

# Sources
set(cameras_files_test
    Camera_Pinhole_Brown_test.cpp
    Camera_Pinhole_Fisheye1_test.cpp
    Camera_Pinhole_Fisheye_test.cpp
    Camera_Pinhole_Radial_test.cpp
)

add_library(aliceVision_cameras INTERFACE)

target_link_libraries(aliceVision_cameras
  INTERFACE aliceVision_multiview
            ${LOG_LIB}
)

UNIT_TEST(aliceVision Camera_Pinhole_Radial   "aliceVision_cameras")
UNIT_TEST(aliceVision Camera_Pinhole_Brown    "aliceVision_cameras")
UNIT_TEST(aliceVision Camera_Pinhole_Fisheye  "aliceVision_cameras")
UNIT_TEST(aliceVision Camera_Pinhole_Fisheye1 "aliceVision_cameras")

add_custom_target(aliceVision_cameras_ide SOURCES ${cameras_files_headers} ${cameras_files_test})

