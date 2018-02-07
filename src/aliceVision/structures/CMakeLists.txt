# Headers
set(structures_files_headers
  jetColorMap.hpp
  image.hpp
  mv_color.hpp
  mv_double3d.hpp
  mv_geometry.hpp
  mv_geometry_triTri.hpp
  mv_matrix.hpp
  mv_matrix2x2.hpp
  mv_matrix3x3.hpp
  mv_matrix3x4.hpp
  mv_point2d.hpp
  mv_point3d.hpp
  mv_staticVector.hpp
  mv_structures.hpp
  mv_universe.hpp
  quaternion.hpp
)

# Sources
set(structures_files_sources
  jetColorMap.cpp
  image.cpp
  mv_color.cpp
  mv_double3d.cpp
  mv_geometry.cpp
  mv_geometry_triTri.cpp
  mv_matrix.cpp
  mv_matrix2x2.cpp
  mv_matrix3x3.cpp
  mv_matrix3x4.cpp
  mv_point2d.cpp
  mv_point3d.cpp
  mv_staticVector.cpp
  mv_structures.cpp
  mv_universe.cpp
  quaternion.cpp
)

add_library(mv_structures
  ${structures_files_headers}
  ${structures_files_sources}
)

target_include_directories(mv_structures
  PUBLIC
  $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${generatedDir}>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIR} ${ZLIB_INCLUDE_DIR}
)

target_link_libraries(mv_structures
  PUBLIC ${ZLIB_LIBRARIES}
         ${Boost_FILESYSTEM_LIBRARIES}
)

set_property(TARGET mv_structures
  PROPERTY FOLDER AliceVision
)
