set(SOURCES_files_planeSweeping
  ps_depthSimMap.cpp
  ps_depthSimMap.hpp
  ps_rctc.cpp
  ps_refine_rc.cpp
  ps_refine_rc.hpp
  ps_sgm_params.cpp
  ps_sgm_params.hpp
  ps_sgm_rc.cpp
  ps_sgm_rc.hpp
  ps_sgm_rctc.cpp
  ps_sgm_rctc.hpp
  ps_sgm_vol.cpp
  ps_sgm_vol.hpp
)
source_group("planeSweeping" FILES ${SOURCES_files_planeSweeping})


add_library(mv_planeSweeping ${SOURCES_files_planeSweeping})
target_include_directories(mv_planeSweeping
         PUBLIC 
         $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>)

target_link_libraries(mv_planeSweeping PUBLIC mv_structures mv_mesh)
