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

# Cuda Headers

# Files excluded from compilation
set(planeSweeping_cuda_files_headers
  # Cuda
  cuda/plane_sweeping/device_code.cu
  cuda/plane_sweeping/device_code_refine.cu
  cuda/plane_sweeping/device_code_volume.cu
  cuda/plane_sweeping/device_code_fuse.cu
  cuda/plane_sweeping/device_utils.cu
)

set(planeSweeping_cuda_deviceCommon_files_headers
  # Headers
  cuda/device_common/device_patch_es_glob.hpp
  # Cuda
  cuda/device_common/device_color.cu
  cuda/device_common/device_eig33.cu
  cuda/device_common/device_global.cu
  cuda/device_common/device_matrix.cu
  cuda/device_common/device_patch_es.cu
  cuda/device_common/device_simStat.cu
)

set_source_files_properties(${planeSweeping_cuda_files_headers}
  PROPERTIES HEADER_FILE_ONLY true
)

set_source_files_properties(${planeSweeping_cuda_deviceCommon_files_headers}
  PROPERTIES HEADER_FILE_ONLY true
)

# Cuda Sources

set(planeSweeping_cuda_files_sources
  cuda/common_gpu_cpu_structures.hpp
  cuda/cuda_plane_sweeping.cpp
  cuda/cuda_plane_sweeping.hpp
  cuda/plane_sweeping/plane_sweeping_cuda.cu
  ${planeSweeping_cuda_files_headers}
  ${planeSweeping_cuda_deviceCommon_files_headers}
)
source_group("planeSweeping_cuda" FILES ${planeSweeping_cuda_files_sources})

cuda_add_library(aliceVision_planeSweeping
  ${planeSweeping_files_headers}
  ${planeSweeping_files_sources}
  ${planeSweeping_cuda_files_sources}
)

target_include_directories(aliceVision_planeSweeping
  PUBLIC $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
)

# TODO : PUBLIC
target_link_libraries(aliceVision_planeSweeping
  aliceVision_structures
  aliceVision_imageIO
  aliceVision_common
)

set_property(TARGET aliceVision_planeSweeping
  PROPERTY FOLDER AliceVision
)
