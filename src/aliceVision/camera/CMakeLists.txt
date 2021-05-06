# Headers
set(camera_files_headers
	camera.hpp
	cameraCommon.hpp
	cameraUndistortImage.hpp
	Distortion.hpp
	Distortion3DE.hpp
	DistortionBrown.hpp
	DistortionFisheye.hpp
	DistortionFisheye1.hpp
	DistortionRadial.hpp
	Equidistant.hpp
	EquidistantRadial.hpp
	IntrinsicBase.hpp
	IntrinsicInitMode.hpp
	IntrinsicsScaleOffset.hpp
	IntrinsicsScaleOffsetDisto.hpp
	Pinhole.hpp
	Pinhole3DE.hpp
	PinholeBrown.hpp
	PinholeFisheye.hpp
	PinholeFisheye1.hpp
	PinholeRadial.hpp
)

alicevision_add_interface(aliceVision_camera
  SOURCES ${camera_files_headers}
  LINKS
    aliceVision_numeric
    aliceVision_geometry
    aliceVision_image
    aliceVision_stl
    aliceVision_system
)

# Unit tests
alicevision_add_test(distortion_test.cpp        NAME "camera_distortionRadial"    LINKS aliceVision_camera)
alicevision_add_test(pinholeBrown_test.cpp      NAME "camera_pinholeBrown"        LINKS aliceVision_camera)
alicevision_add_test(pinholeFisheye_test.cpp    NAME "camera_pinholeFisheye"      LINKS aliceVision_camera)
alicevision_add_test(pinholeFisheye1_test.cpp   NAME "camera_pinholeFisheye1"     LINKS aliceVision_camera)
alicevision_add_test(pinholeRadial_test.cpp     NAME "camera_pinholeRadial"       LINKS aliceVision_camera)
alicevision_add_test(pinhole3DE_test.cpp     	NAME "camera_pinhole3DE"       LINKS aliceVision_camera)
alicevision_add_test(equidistant_test.cpp       NAME "camera_equidistant"         LINKS aliceVision_camera)
