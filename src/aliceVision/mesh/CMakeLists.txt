set(SOURCES_files_mesh
  mv_mesh.cpp
  mv_mesh.hpp
  meshVisibility.cpp
  meshVisibility.hpp
  mv_mesh_analyze.cpp
  mv_mesh_analyze.hpp
  mv_mesh_clean.cpp
  mv_mesh_clean.hpp
  mv_mesh_energy_opt.cpp
  mv_mesh_energy_opt.hpp
  mv_mesh_energy_opt_photo_mem.cpp
  mv_mesh_energy_opt_photo_mem.hpp
  mv_mesh_refine.cpp
  mv_mesh_retexture_obj.cpp
  mv_mesh_retexture_obj.hpp
  mv_mesh_uvatlas.cpp
  mv_mesh_uvatlas.hpp
)
source_group("mesh" FILES ${SOURCES_files_mesh})


add_library(mv_mesh ${SOURCES_files_mesh})
target_include_directories(mv_mesh
         PUBLIC 
         $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
         ${Boost_INCLUDE_DIR})

target_link_libraries(mv_mesh PUBLIC mv_structures aliceVision_common aliceVision_imageIO mv_planeSweeping mv_output3D mv_rply mv_cudaInterfaces mv_delaunaycut ${Boost_FILESYSTEM_LIBRARIES})
