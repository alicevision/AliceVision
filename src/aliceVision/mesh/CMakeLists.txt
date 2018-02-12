# Headers
set(mesh_files_headers
  mv_mesh.hpp
  mv_mesh_analyze.hpp
  mv_mesh_clean.hpp
  mv_mesh_energy_opt.hpp
  mv_mesh_energy_opt_photo_mem.hpp
  mv_mesh_retexture_obj.hpp
  mv_mesh_uvatlas.hpp
  meshVisibility.hpp
  mv_plyloader.hpp
  mv_plysaver.hpp
  rply.h
)

# Sources
set(mesh_files_sources
  mv_mesh.cpp
  mv_mesh_analyze.cpp
  mv_mesh_clean.cpp
  mv_mesh_energy_opt.cpp
  mv_mesh_energy_opt_photo_mem.cpp
  mv_mesh_refine.cpp
  mv_mesh_retexture_obj.cpp
  mv_mesh_uvatlas.cpp
  meshVisibility.cpp
  mv_plyloader.cpp
  mv_plysaver.cpp
  rply.c
)

add_library(aliceVision_mesh
  ${mesh_files_headers}
  ${mesh_files_sources}
)

target_include_directories(aliceVision_mesh
  PUBLIC
  $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
  $<BUILD_INTERFACE:${generatedDir}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(aliceVision_mesh
  PUBLIC mv_structures
         aliceVision_common
         aliceVision_imageIO
         mv_planeSweeping
         mv_cudaInterfaces
         ${Boost_FILESYSTEM_LIBRARIES}
         Geogram::geogram
)

set_property(TARGET aliceVision_mesh
  PROPERTY FOLDER AliceVision
)


