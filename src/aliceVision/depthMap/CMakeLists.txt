# Headers
set(depthMap_files_headers
  DepthSimMap.hpp
  RcTc.hpp
  RefineRc.hpp
  SemiGlobalMatchingParams.hpp
  SemiGlobalMatchingRc.hpp
  SemiGlobalMatchingRcTc.hpp
  SemiGlobalMatchingVolume.hpp
)

# Sources
set(depthMap_files_sources
  DepthSimMap.cpp
  RcTc.cpp
  RefineRc.cpp
  SemiGlobalMatchingParams.cpp
  SemiGlobalMatchingRc.cpp
  SemiGlobalMatchingRcTc.cpp
  SemiGlobalMatchingVolume.cpp
)

# Cuda Sources
set(depthMap_cuda_files_sources
  cuda/commonStructures.cpp                 cuda/commonStructures.hpp
  cuda/PlaneSweepingCuda.cpp                cuda/PlaneSweepingCuda.hpp
  cuda/planeSweeping/plane_sweeping_cuda.cu cuda/planeSweeping/plane_sweeping_cuda.hpp
  cuda/planeSweeping/cuda_global_data.cu    cuda/planeSweeping/cuda_global_data.cuh
  cuda/planeSweeping/device_code_refine.cu  cuda/planeSweeping/device_code_refine.cuh
  cuda/planeSweeping/device_utils.cuh
  cuda/planeSweeping/device_code.cu         cuda/planeSweeping/device_code.cuh
  cuda/planeSweeping/device_code_volume.cu  cuda/planeSweeping/device_code_volume.cuh
  cuda/planeSweeping/device_code_fuse.cu    cuda/planeSweeping/device_code_fuse.cuh
  cuda/deviceCommon/device_patch_es.cuh
  cuda/deviceCommon/device_color.cu         cuda/deviceCommon/device_color.cuh
  cuda/deviceCommon/device_eig33.cuh
  cuda/deviceCommon/device_global.cu        cuda/deviceCommon/device_global.cuh
  cuda/deviceCommon/device_matrix.cuh
  cuda/deviceCommon/device_simStat.cuh
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
    aliceVision_imageIO
    aliceVision_mvsUtils
    aliceVision_system
    ${Boost_FILESYSTEM_LIBRARY}
    ${CUDA_CUDADEVRT_LIBRARY}
    ${CUDA_CUBLAS_LIBRARIES} #TODO shouldn't be here, but required to build on some machines
  PUBLIC_INCLUDE_DIRS
    ${CUDA_INCLUDE_DIRS}
)
