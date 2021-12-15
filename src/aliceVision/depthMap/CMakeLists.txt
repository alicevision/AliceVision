# Headers
set(depthMap_files_headers
  computeOnMultiGPUs.hpp
  depthMap.hpp
  DepthSimMap.hpp
  Refine.hpp
  RefineParams.hpp
  Sgm.hpp
  SgmParams.hpp
  volumeIO.hpp
)

# Sources
set(depthMap_files_sources
  computeOnMultiGPUs.cpp
  depthMap.cpp
  DepthSimMap.cpp
  Refine.cpp
  Sgm.cpp
  SgmParams.cpp
  volumeIO.cpp
)

# Cuda Headers
set(depthMap_cuda_files_headers
  # Headers
  cuda/deviceCommon/device_patch_es_glob.hpp
  cuda/planeSweeping/host_utils.h
  cuda/planeSweeping/plane_sweeping_cuda.hpp
  # deviceCommon
  cuda/deviceCommon/device_color.cu
  cuda/deviceCommon/device_global.cu
  cuda/deviceCommon/device_matrix.cu
  cuda/deviceCommon/device_matrix.cuh
  cuda/deviceCommon/device_patch_es.cu
  cuda/deviceCommon/device_simStat.cu
  cuda/deviceCommon/device_operators.cuh
  cuda/deviceCommon/device_utils.cuh
  cuda/deviceCommon/device_utils.h
  # planeSweeping
  cuda/planeSweeping/device_code.cu
  cuda/planeSweeping/device_code_refine.cu
  cuda/planeSweeping/device_code_volume.cu
  cuda/planeSweeping/device_code_fuse.cu
  # normalmap
  cuda/normalmap/device_eig33.cuh
)

set_source_files_properties(${depthMap_cuda_files_headers}
  PROPERTIES HEADER_FILE_ONLY true
)

# Cuda Sources
set(depthMap_cuda_files_sources
  cuda/commonStructures.hpp
  cuda/FrameCacheMemory.cpp
  cuda/FrameCacheMemory.hpp
  cuda/LRUCache.hpp
  cuda/OneTC.hpp
  cuda/PlaneSweepingCuda.cpp
  cuda/PlaneSweepingCuda.hpp
  cuda/planeSweeping/plane_sweeping_cuda.hpp
  cuda/planeSweeping/plane_sweeping_cuda.cu
  cuda/normalmap/normal_map.hpp
  cuda/normalmap/normal_map.cu
  cuda/images/gauss_filter.hpp
  cuda/images/gauss_filter.cu
  ${depthMap_cuda_files_headers}
)

source_group("aliceVision_depthMap_cuda" FILES ${depthMap_cuda_files_sources})

alicevision_add_library(aliceVision_depthMap
  USE_CUDA
  SOURCES
    ${depthMap_files_headers}
    ${depthMap_files_sources}
    ${depthMap_cuda_files_sources}
  PUBLIC_LINKS
    aliceVision_mvsData
    aliceVision_mvsUtils
    aliceVision_system
    Boost::filesystem
    ${CUDA_CUDADEVRT_LIBRARY}
    ${CUDA_CUBLAS_LIBRARIES} #TODO shouldn't be here, but required to build on some machines
  PRIVATE_LINKS
    aliceVision_gpu
    aliceVision_sfmData
    aliceVision_sfmDataIO
  PUBLIC_INCLUDE_DIRS
    ${CUDA_INCLUDE_DIRS}
)

# target_compile_definitions(aliceVision_depthMap PUBLIC TSIM_USE_FLOAT)

