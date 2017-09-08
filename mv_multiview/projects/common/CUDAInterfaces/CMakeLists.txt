
set(SOURCES_files_cudaInterfaces_plane_sweeping
  plane_sweeping/plane_sweeping_cuda.cu
)

# Files excluded from compilation
set(HEADER_files_cudaInterfaces_plane_sweeping
  # Cuda
  plane_sweeping/device_code.cu
  plane_sweeping/device_code_refine.cu
  plane_sweeping/device_code_volume.cu
  plane_sweeping/device_code_fuse.cu
  plane_sweeping/device_utils.cu
)
set_source_files_properties(${HEADER_files_cudaInterfaces_plane_sweeping} PROPERTIES HEADER_FILE_ONLY true)
 
# Files excluded from compilation
set(HEADER_files_cudaInterfaces_device_common
  # Headers
  device_common/device_patch_es_glob.h
  # Cuda
  device_common/device_color.cu
  device_common/device_eig33.cu
  device_common/device_global.cu
  device_common/device_matrix.cu
  device_common/device_patch_es.cu
  device_common/device_simStat.cu
)
set_source_files_properties(${HEADER_files_cudaInterfaces_device_common} PROPERTIES HEADER_FILE_ONLY true)

set(SOURCES_files_cudaInterfaces
  common_gpu_cpu_structures.h
  cuda_plane_sweeping.cpp
  cuda_plane_sweeping.h
  refine.cpp
  refine.h
  ${SOURCES_files_cudaInterfaces_plane_sweeping}
  ${HEADER_files_cudaInterfaces_plane_sweeping}
  ${HEADER_files_cudaInterfaces_device_common}
)
source_group("cudaInterfaces" FILES ${SOURCES_files_cudaInterfaces})

cuda_add_library(mv_cudaInterfaces ${SOURCES_files_cudaInterfaces})

target_include_directories(mv_cudaInterfaces 
         PUBLIC 
         $<BUILD_INTERFACE:${CMPMVS_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>)

# PUBLIC
target_link_libraries(mv_cudaInterfaces mv_structures mv_prematching mv_mesh mv_output3D mv_delaunaycut)
