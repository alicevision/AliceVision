# Headers
set(camera_files_headers
	camera.hpp
	cameraCommon.hpp
	cameraUndistortImage.hpp
	Distortion.hpp
	DistortionBrown.hpp
	DistortionFisheye.hpp
	DistortionFisheye1.hpp
	DistortionRadial.hpp
	Undistortion.hpp
	Undistortion3DEA4.hpp
    Undistortion3DERadial4.hpp
    Undistortion3DEClassicLD.hpp
    UndistortionRadial.hpp
	Equidistant.hpp
    Equirectangular.hpp
	IntrinsicBase.hpp
	IntrinsicInitMode.hpp
	IntrinsicScaleOffset.hpp
	IntrinsicScaleOffsetDisto.hpp
	Pinhole.hpp
)

set(camera_files_sources
    DistortionBrown.cpp
    DistortionFisheye.cpp
    DistortionFisheye1.cpp
    DistortionRadial.cpp
    UndistortionRadial.cpp
    Equidistant.cpp
    Equirectangular.cpp
    IntrinsicBase.cpp
    IntrinsicScaleOffset.cpp
    IntrinsicScaleOffsetDisto.cpp
    Pinhole.cpp
	Undistortion.cpp
    Undistortion3DEA4.cpp
    Undistortion3DERadial4.cpp
    Undistortion3DEClassicLD.cpp
)

alicevision_add_library(aliceVision_camera
    SOURCES ${camera_files_headers} ${camera_files_sources}
    PUBLIC_LINKS
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
alicevision_add_test(equidistant_test.cpp       NAME "camera_equidistant"         LINKS aliceVision_camera)


# SWIG Binding
if (ALICEVISION_BUILD_SWIG_BINDING)
    alicevision_swig_add_library(camera
        SOURCES Camera.i
        PUBLIC_LINKS
            aliceVision_camera
            ${Python3_LIBRARIES}
        PRIVATE_INCLUDE_DIRS
            ../include
            ${ALICEVISION_ROOT}/include
            ${Python3_INCLUDE_DIRS}
            ${Python3_NumPy_INCLUDE_DIRS}
    )
endif()