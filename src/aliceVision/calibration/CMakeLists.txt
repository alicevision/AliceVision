# Headers
set(calibration_files_headers

    distortionEstimation.hpp
    distortionEstimationLine.hpp
    distortionEstimationPoint.hpp
    distortionEstimationGeometry.hpp
    checkerDetector.hpp
    checkerDetector_io.hpp
)

# Sources
set(calibration_files_sources
    distortionEstimationLine.cpp
    distortionEstimationPoint.cpp
    distortionEstimationGeometry.cpp
    checkerDetector.cpp
    checkerDetector_io.cpp
)


alicevision_add_library(aliceVision_calibration
    SOURCES ${calibration_files_headers} ${calibration_files_sources}
    PRIVATE_LINKS
        aliceVision_image
        aliceVision_system
        aliceVision_dataio
        Boost::program_options
        Boost::boost
        Boost::json
        ${CERES_LIBRARIES}
)
