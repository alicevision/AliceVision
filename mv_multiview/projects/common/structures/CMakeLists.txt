
set(SOURCES_files_structures
  mv_color.cpp
  mv_color.h
  mv_common.cpp
  mv_common.h
  mv_double3d.cpp
  mv_double3d.h
  mv_filesio.cpp
  mv_filesio.h
  mv_geometry.cpp
  mv_geometry.h
  mv_geometry_triTri.cpp
  mv_geometry_triTri.h
  mv_images_cache.cpp
  mv_images_cache.h
  mv_matrix.cpp
  mv_matrix.h
  mv_matrix2x2.cpp
  mv_matrix2x2.h
  mv_matrix3x3.cpp
  mv_matrix3x3.h
  mv_matrix3x4.cpp
  mv_matrix3x4.h
  mv_multiview_params.cpp
  mv_multiview_params.h
  mv_point2d.cpp
  mv_point2d.h
  mv_point3d.cpp
  mv_point3d.h
  mv_staticVector.cpp
  mv_staticVector.h
  mv_structures.cpp
  mv_structures.h
  mv_universe.cpp
  mv_universe.h
  quaternion.cpp
  quaternion.h
)
source_group("structures" FILES ${SOURCES_files_structures})

# message(WARNING "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
# message(WARNING "OpenCV_LIBS: ${OpenCV_LIBS}")

add_library(mv_structures ${SOURCES_files_structures})
target_include_directories(mv_structures
         PUBLIC
         $<BUILD_INTERFACE:${CMPMVS_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
         ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${ZLIB_INCLUDE_DIR})

target_link_libraries(mv_structures PUBLIC eig33 ${ZLIB_LIBRARIES} ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARIES})
