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
    ConstraintPoint.hpp
    Constraint2D.hpp
    SurveyPoint.hpp
)

# Sources
set(sfmData_files_sources
    SfMData.cpp
    uid.cpp
    View.cpp
    colorize.cpp
    exif.cpp
    ImageInfo.cpp
    Landmark.cpp
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
        aliceVision_camera
        aliceVision_system
)
alicevision_add_test(view_test.cpp
    NAME "view"
    LINKS aliceVision_sfmData
)

# SWIG Binding
if (ALICEVISION_BUILD_SWIG_BINDING)
    alicevision_swig_add_library(sfmData
        SOURCES SfMData.i
        PUBLIC_LINKS
            aliceVision_sfmData
            aliceVision_camera
            ${Python3_LIBRARIES}
        PRIVATE_INCLUDE_DIRS
            ../include
            ${ALICEVISION_ROOT}/include
            ${Python3_INCLUDE_DIRS}
            ${Python3_NumPy_INCLUDE_DIRS}
    )
endif()
