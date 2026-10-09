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

# Unit tests
alicevision_add_test(checkerDetector_test.cpp NAME "calibration_checkerDetector" LINKS aliceVision_calibration aliceVision_image)
alicevision_add_test(checkerDetector_io_test.cpp NAME "calibration_checkerDetectorIO" LINKS aliceVision_calibration aliceVision_numeric Boost::json)
alicevision_add_test(distortionEstimationPoint_test.cpp NAME "calibration_distortionEstimationPoint" LINKS aliceVision_calibration aliceVision_camera aliceVision_numeric)
alicevision_add_test(distortionEstimationLine_test.cpp NAME "calibration_distortionEstimationLine" LINKS aliceVision_calibration aliceVision_camera aliceVision_numeric)
alicevision_add_test(distortionEstimationGeometry_test.cpp NAME "calibration_distortionEstimationGeometry" LINKS aliceVision_calibration aliceVision_camera aliceVision_numeric)
