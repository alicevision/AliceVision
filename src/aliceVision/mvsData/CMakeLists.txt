# Headers
set(structures_files_headers
  Color.hpp
  geometry.hpp
  geometryTriTri.hpp
  image.hpp
  jetColorMap.hpp
  Matrix3x3.hpp
  Matrix3x4.hpp
  OrientedPoint.hpp
  Point2d.hpp
  Point3d.hpp
  Point4d.hpp
  Pixel.hpp
  Rgb.hpp
  SeedPoint.hpp
  SeedPointCams.hpp
  Stat3d.hpp
  StaticVector.hpp
  structures.hpp
  Universe.hpp
  Voxel.hpp
)

# Sources
set(structures_files_sources
  jetColorMap.cpp
  image.cpp
  geometry.cpp
  geometryTriTri.cpp
  Stat3d.cpp
  StaticVector.cpp
  structures.cpp
  Universe.cpp
)

add_library(aliceVision_mvsData
  ${structures_files_headers}
  ${structures_files_sources}
)

target_include_directories(aliceVision_mvsData
  PUBLIC $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
         ${ZLIB_INCLUDE_DIR}
         ${Boost_INCLUDE_DIR}
)

target_link_libraries(aliceVision_mvsData
  PUBLIC 
    aliceVision_system
    ${ZLIB_LIBRARIES}
    ${Boost_FILESYSTEM_LIBRARY}
)

set_property(TARGET aliceVision_mvsData
  PROPERTY FOLDER AliceVision
)

install(TARGETS aliceVision_mvsData
  DESTINATION lib
  EXPORT aliceVision-targets
)
