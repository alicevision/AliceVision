# Headers
set(camera_files_headers
	camera.hpp
	cameraCommon.hpp
	cameraUndistortImage.hpp
	IntrinsicBase.hpp
	IntrinsicInitMode.hpp
	Pinhole.hpp
	PinholeBrown.hpp
	PinholeFisheye.hpp
	PinholeFisheye1.hpp
	PinholeRadial.hpp
)

alicevision_add_interface(aliceVision_camera
  SOURCES ${camera_files_headers}
  LINKS
    aliceVision_geometryData
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
