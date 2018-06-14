# Headers
set(camera_files_headers
	camera.hpp
	cameraCommon.hpp
	cameraUndistortImage.hpp
	IntrinsicBase.hpp
	Pinhole.hpp
	PinholeBrown.hpp
	PinholeFisheye.hpp
	PinholeFisheye1.hpp
	PinholeRadial.hpp
)

# Sources
set(camera_files_test
  pinholeBrown_test.cpp
  pinholeFisheye1_test.cpp
  pinholeFisheye_test.cpp
  pinholeRadial_test.cpp
)

add_library(aliceVision_camera INTERFACE)

target_link_libraries(aliceVision_camera
  INTERFACE 
    aliceVision_geometry
    aliceVision_multiview
    aliceVision_image
    aliceVision_numeric
    aliceVision_stl
    aliceVision_system
)

install(TARGETS aliceVision_camera 
  EXPORT aliceVision-targets 
)

UNIT_TEST(aliceVision pinholeBrown    "aliceVision_camera")
UNIT_TEST(aliceVision pinholeFisheye  "aliceVision_camera")
UNIT_TEST(aliceVision pinholeFisheye1 "aliceVision_camera")
UNIT_TEST(aliceVision pinholeRadial   "aliceVision_camera")

add_custom_target(aliceVision_camera_ide SOURCES ${camera_files_headers} ${camera_files_test})

set_property(TARGET aliceVision_camera_ide
  PROPERTY FOLDER AliceVision
)
