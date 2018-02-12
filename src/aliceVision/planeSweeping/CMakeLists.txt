# Headers
set(planeSweeping_files_headers
  ps_depthSimMap.hpp
  ps_refine_rc.hpp
  ps_sgm_params.hpp
  ps_sgm_rc.hpp
  ps_sgm_rctc.hpp
  ps_sgm_vol.hpp
)

# Sources
set(planeSweeping_files_sources
  ps_depthSimMap.cpp
  ps_rctc.cpp
  ps_refine_rc.cpp
  ps_sgm_params.cpp
  ps_sgm_rc.cpp
  ps_sgm_rctc.cpp
  ps_sgm_vol.cpp
)

add_library(aliceVision_planeSweeping
  ${planeSweeping_files_headers}
  ${planeSweeping_files_sources}
)

target_include_directories(aliceVision_planeSweeping
  PUBLIC $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
)

target_link_libraries(aliceVision_planeSweeping
  PUBLIC aliceVision_structures
         aliceVision_imageIO
         aliceVision_common
         aliceVision_cudaInterfaces
)

set_property(TARGET aliceVision_planeSweeping
  PROPERTY FOLDER AliceVision
)
