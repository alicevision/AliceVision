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
    set(UseSWIG_TARGET_NAME_PREFERENCE STANDARD)
    set_property(SOURCE Camera.i PROPERTY CPLUSPLUS ON)
    set_property(SOURCE Camera.i PROPERTY SWIG_MODULE_NAME camera)

    swig_add_library(camera
        TYPE MODULE
        LANGUAGE python
        SOURCES Camera.i
    )

    if (CMAKE_SYSTEM_NAME MATCHES "Linux")
        list(APPEND SWIG_EXTRA_COMPILE_OPTIONS "-DSWIGWORDSIZE64")
    endif()

    set_property(
        TARGET camera
        PROPERTY SWIG_COMPILE_OPTIONS -doxygen ${SWIG_EXTRA_COMPILE_OPTIONS}
    )

    target_include_directories(camera
    PRIVATE
        ../include
        ${ALICEVISION_ROOT}/include
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
    )
    set_property(
        TARGET camera
        PROPERTY SWIG_USE_TARGET_INCLUDE_DIRECTORIES ON
    )
    set_property(
        TARGET camera
        PROPERTY COMPILE_OPTIONS -std=c++17
    )

    target_link_libraries(camera
    PUBLIC
        aliceVision_camera
        ${Python3_LIBRARIES}
    )

    install(
    TARGETS
    camera
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
    install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/camera.py
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
endif()