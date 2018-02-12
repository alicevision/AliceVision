# planeSweeping

# Sources
set(cudaInterfaces_planeSweeping_files_sources
  plane_sweeping/plane_sweeping_cuda.cu
)

# Files excluded from compilation
# Headers
set(cudaInterfaces_planeSweeping_files_headers
  # Cuda
  plane_sweeping/device_code.cu
  plane_sweeping/device_code_refine.cu
  plane_sweeping/device_code_volume.cu
  plane_sweeping/device_code_fuse.cu
  plane_sweeping/device_utils.cu
)

set_source_files_properties(${cudaInterfaces_planeSweeping_files_headers}
  PROPERTIES HEADER_FILE_ONLY true
)

# deviceCommon
 
# Files excluded from compilation
set(cudaInterfaces_deviceCommon_files_headers
  # Headers
  device_common/device_patch_es_glob.hpp
  # Cuda
  device_common/device_color.cu
  device_common/device_eig33.cu
  device_common/device_global.cu
  device_common/device_matrix.cu
  device_common/device_patch_es.cu
  device_common/device_simStat.cu
)

set_source_files_properties(${cudaInterfaces_deviceCommon_files_headers}
  PROPERTIES HEADER_FILE_ONLY true
)

# cudaInterfaces

set(cudaInterfaces_files_sources
  common_gpu_cpu_structures.hpp
  cuda_plane_sweeping.cpp
  cuda_plane_sweeping.hpp
  ${cudaInterfaces_planeSweeping_files_sources}
  ${cudaInterfaces_planeSweeping_files_headers}
  ${cudaInterfaces_deviceCommon_files_headers}
)

source_group("cudaInterfaces" FILES ${cudaInterfaces_files_sources})

cuda_add_library(aliceVision_cudaInterfaces ${cudaInterfaces_files_sources})

target_include_directories(aliceVision_cudaInterfaces
   PUBLIC $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
          $<BUILD_INTERFACE:${generatedDir}>
          $<INSTALL_INTERFACE:include>
)

# TODO: PUBLIC
target_link_libraries(aliceVision_cudaInterfaces
  aliceVision_common
  aliceVision_imageIO
)
