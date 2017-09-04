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
set(cameras_files_test
    Camera_Pinhole_Brown_test.cpp
    Camera_Pinhole_Fisheye1_test.cpp
    Camera_Pinhole_Fisheye_test.cpp
    Camera_Pinhole_Radial_test.cpp
)

add_library(openMVG_cameras INTERFACE)
target_sources(openMVG_cameras INTERFACE
    ${cameras_files_headers}
    )


target_link_libraries(openMVG_cameras
  INTERFACE openMVG_multiview
            ${LOG_LIB}
)

UNIT_TEST(openMVG Camera_Pinhole_Radial   "openMVG_cameras")
UNIT_TEST(openMVG Camera_Pinhole_Brown    "openMVG_cameras")
UNIT_TEST(openMVG Camera_Pinhole_Fisheye  "openMVG_cameras")
UNIT_TEST(openMVG Camera_Pinhole_Fisheye1 "openMVG_cameras")

add_custom_target(openMVG_cameras_ide SOURCES ${cameras_files_headers} ${cameras_files_test})

