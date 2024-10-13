# Headers
set(sfmData_files_headers
  SfMData.hpp
  CameraPose.hpp
  Landmark.hpp
  View.hpp
  Rig.hpp
  uid.hpp
  colorize.hpp
  exif.hpp
  ImageInfo.hpp
  ExposureSetting.hpp
  Observation.hpp
)

# Sources
set(sfmData_files_sources
  SfMData.cpp
  uid.cpp
  View.cpp
  colorize.cpp
  exif.cpp
  ImageInfo.cpp
  Observation.cpp
)

alicevision_add_library(aliceVision_sfmData
  SOURCES ${sfmData_files_headers} ${sfmData_files_sources}
  PUBLIC_LINKS
    aliceVision_system
    aliceVision_numeric
    aliceVision_image
    aliceVision_feature
    aliceVision_geometry
    aliceVision_camera
    aliceVision_sensorDB
    aliceVision_stl
  PRIVATE_LINKS
    Boost::boost
)

# Unit tests

alicevision_add_test(sfmData_test.cpp
  NAME "sfmData"
  LINKS aliceVision_sfmData
        aliceVision_system
)
alicevision_add_test(view_test.cpp
  NAME "view"
  LINKS aliceVision_sfmData
)

# SWIG Binding
if (ALICEVISION_BUILD_SWIG_BINDING)
    set(UseSWIG_TARGET_NAME_PREFERENCE STANDARD)
    set_property(SOURCE SfMData.i PROPERTY CPLUSPLUS ON)
    set_property(SOURCE SfMData.i PROPERTY SWIG_MODULE_NAME sfmData)

    swig_add_library(sfmData
        TYPE MODULE
        LANGUAGE python
        SOURCES SfMData.i
    )

    if (CMAKE_SYSTEM_NAME MATCHES "Linux")
        list(APPEND SWIG_EXTRA_COMPILE_OPTIONS "-DSWIGWORDSIZE64")
    endif()

    set_property(
        TARGET sfmData
        PROPERTY SWIG_COMPILE_OPTIONS -doxygen ${SWIG_EXTRA_COMPILE_OPTIONS}
    )

    target_include_directories(sfmData
    PRIVATE
        ../include
        ${ALICEVISION_ROOT}/include
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
    )
    set_property(
        TARGET sfmData
        PROPERTY SWIG_USE_TARGET_INCLUDE_DIRECTORIES ON
    )
    set_property(
        TARGET sfmData
        PROPERTY COMPILE_OPTIONS -std=c++17
    )

    target_link_libraries(sfmData
    PUBLIC
        aliceVision_sfmData
        aliceVision_camera
        ${Python3_LIBRARIES}
    )

    install(
    TARGETS
        sfmData
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
    install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/sfmData.py
    DESTINATION
        ${CMAKE_INSTALL_PREFIX}
    )
endif()