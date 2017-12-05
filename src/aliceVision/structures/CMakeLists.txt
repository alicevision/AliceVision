set(SOURCES_files_structures
  mv_color.cpp
  mv_color.hpp
  mv_double3d.cpp
  mv_double3d.hpp
  mv_geometry.cpp
  mv_geometry.hpp
  mv_geometry_triTri.cpp
  mv_geometry_triTri.hpp
  mv_matrix.cpp
  mv_matrix.hpp
  mv_matrix2x2.cpp
  mv_matrix2x2.hpp
  mv_matrix3x3.cpp
  mv_matrix3x3.hpp
  mv_matrix3x4.cpp
  mv_matrix3x4.hpp
  mv_point2d.cpp
  mv_point2d.hpp
  mv_point3d.cpp
  mv_point3d.hpp
  mv_staticVector.cpp
  mv_staticVector.hpp
  mv_structures.cpp
  mv_structures.hpp
  mv_universe.cpp
  mv_universe.hpp
  quaternion.cpp
  quaternion.hpp
)
source_group("structures" FILES ${SOURCES_files_structures})

# message(WARNING "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
# message(WARNING "OpenCV_LIBS: ${OpenCV_LIBS}")

add_library(mv_structures ${SOURCES_files_structures})
target_include_directories(mv_structures
         PUBLIC
         $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
         ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${ZLIB_INCLUDE_DIR})

target_link_libraries(mv_structures PUBLIC eig33 ${ZLIB_LIBRARIES} ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARIES})
