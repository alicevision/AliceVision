
# Headers
set(hdr_files_headers
 LaguerreBACalibration.hpp
 sampling.hpp
 rgbCurve.hpp
 hdrMerge.hpp
 DebevecCalibrate.hpp
 GrossbergCalibrate.hpp
 emorCurve.hpp
 QuadProg++.hpp
)

# Sources
set(hdr_files_sources
 LaguerreBACalibration.cpp
 sampling.cpp
 rgbCurve.cpp
 hdrMerge.cpp
 DebevecCalibrate.cpp
 GrossbergCalibrate.cpp
 emorCurve.cpp
 emorCurveInv.cpp
 QuadProg++.cpp
)

alicevision_add_library(aliceVision_hdr
  SOURCES ${hdr_files_headers} ${hdr_files_sources}
  PRIVATE_LINKS
    aliceVision_system
    aliceVision_image
    Boost::filesystem
    ${CERES_LIBRARIES}
)

# Unit tests
#alicevision_add_test(hdr_test.cpp      NAME "hdr"            LINKS aliceVision_image aliceVision_hdr)
