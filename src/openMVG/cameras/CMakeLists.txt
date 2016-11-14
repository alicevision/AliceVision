add_library(openMVG_cameras INTERFACE)
target_link_libraries(openMVG_cameras INTERFACE openMVG_multiview ${LOG_LIB})

UNIT_TEST(openMVG Camera_Pinhole_Radial "openMVG_cameras")

UNIT_TEST(openMVG Camera_Pinhole_Brown "openMVG_cameras")

UNIT_TEST(openMVG Camera_Pinhole_Fisheye "openMVG_cameras")

UNIT_TEST(openMVG Camera_Pinhole_Fisheye1 "openMVG_cameras")
