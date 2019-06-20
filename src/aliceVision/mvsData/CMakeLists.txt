# Headers
set(mvsData_files_headers
  Color.hpp
  Image.hpp
  geometry.hpp
  geometryTriTri.hpp
  imageIO.hpp
  jetColorMap.hpp
  Matrix3x3.hpp
  Matrix3x4.hpp
  OrientedPoint.hpp
  Point2d.hpp
  Point3d.hpp
  Point4d.hpp
  Pixel.hpp
  Rgb.hpp
  Stat3d.hpp
  StaticVector.hpp
  structures.hpp
  Universe.hpp
  Voxel.hpp
)

# Sources
set(mvsData_files_sources
  jetColorMap.cpp
  Image.cpp
  imageIO.cpp
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
    ${ZLIB_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
    ${OPENIMAGEIO_LIBRARIES}
  PUBLIC_INCLUDE_DIRS
    ${ZLIB_INCLUDE_DIR}
    ${Boost_INCLUDE_DIR}
    ${OPENIMAGEIO_INCLUDE_DIRS}
)
