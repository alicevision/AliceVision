# Headers
set(mvsData_files_headers
  geometry.hpp
  geometryTriTri.hpp
  Matrix3x3.hpp
  Matrix3x4.hpp
  OrientedPoint.hpp
  Point2d.hpp
  Point3d.hpp
  Point4d.hpp
  Pixel.hpp
  ROI.hpp
  Stat3d.hpp
  StaticVector.hpp
  structures.hpp
  Universe.hpp
  Voxel.hpp
)

# Sources
set(mvsData_files_sources
  geometry.cpp
  geometryTriTri.cpp
  Stat3d.cpp
  StaticVector.cpp
  structures.cpp
  Universe.cpp
)

alicevision_add_library(aliceVision_mvsData
  SOURCES ${mvsData_files_headers} ${mvsData_files_sources}
  PUBLIC_LINKS
    aliceVision_system
    aliceVision_numeric
    aliceVision_image
    ZLIB::ZLIB
    Boost::boost
    OpenImageIO::OpenImageIO
    OpenImageIO::OpenImageIO_Util
)
