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

alicevision_add_interface(aliceVision_camera
  SOURCES ${camera_files_headers} ${camera_files_test}
  LINKS
    aliceVision_geometry
    aliceVision_multiview
    aliceVision_image
    aliceVision_numeric
    aliceVision_stl
    aliceVision_system
)

# Unit tests
alicevision_add_test(pinholeBrown_test.cpp    NAME "camera_pinholeBrown"    LINKS aliceVision_camera)
alicevision_add_test(pinholeFisheye_test.cpp  NAME "camera_pinholeFisheye"  LINKS aliceVision_camera)
alicevision_add_test(pinholeFisheye1_test.cpp NAME "camera_pinholeFisheye1" LINKS aliceVision_camera)
alicevision_add_test(pinholeRadial_test.cpp   NAME "camera_pinholeRadial"   LINKS aliceVision_camera)
