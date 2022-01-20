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
  # deviceCommon
  cuda/deviceCommon/color.cuh
  cuda/deviceCommon/matrix.cuh
  cuda/deviceCommon/operators.cuh
  cuda/deviceCommon/utils.cuh
  cuda/deviceCommon/Patch.cuh
  cuda/deviceCommon/SimStat.cuh
  cuda/deviceCommon/device_utils.h
  # planeSweeping
  cuda/planeSweeping/deviceFuseKernels.cuh
  cuda/planeSweeping/deviceRefineKernels.cuh
  cuda/planeSweeping/deviceSimilarityVolumeKernels.cuh
  cuda/planeSweeping/host_utils.h
  # normalmap
  cuda/normalmap/device_eig33.cuh
)

set_source_files_properties(${depthMap_cuda_files_headers}
  PROPERTIES HEADER_FILE_ONLY true
)

# Cuda Sources
set(depthMap_cuda_files_sources
  cuda/memory.hpp
  cuda/utils.hpp
  cuda/utils.cpp
  cuda/DeviceCache.cpp
  cuda/DeviceCache.hpp
  cuda/DeviceCamera.cpp
  cuda/DeviceCamera.hpp
  cuda/LRUCameraCache.hpp
  cuda/LRUCache.hpp
  cuda/OneTC.hpp
  cuda/PlaneSweepingCuda.cpp
  cuda/PlaneSweepingCuda.hpp
  cuda/planeSweeping/similarity.hpp
  cuda/planeSweeping/deviceFuse.hpp
  cuda/planeSweeping/deviceFuse.cu
  cuda/planeSweeping/deviceRefine.hpp
  cuda/planeSweeping/deviceRefine.cu
  cuda/planeSweeping/deviceSimilarityVolume.hpp
  cuda/planeSweeping/deviceSimilarityVolume.cu
  cuda/normalmap/normal_map.hpp
  cuda/normalmap/normal_map.cu
  cuda/images/gauss_filter.hpp
  cuda/images/gauss_filter.cu
  cuda/images/deviceColorConversion.hpp
  cuda/images/deviceColorConversion.cu
  cuda/deviceCommon/DeviceCameraParams.hpp
  cuda/deviceCommon/DeviceCameraParams.cu
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

