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

# Cuda Headers
set(depthMap_cuda_files_headers
  # Headers
  cuda/deviceCommon/device_patch_es_glob.hpp
  cuda/planeSweeping/host_utils.h
  cuda/planeSweeping/plane_sweeping_cuda.hpp
  # deviceCommon
  cuda/deviceCommon/device_color.cu
  cuda/deviceCommon/device_eig33.cu
  cuda/deviceCommon/device_global.cu
  cuda/deviceCommon/device_matrix.cu
  cuda/deviceCommon/device_patch_es.cu
  cuda/deviceCommon/device_simStat.cu
  cuda/deviceCommon/device_operators.h
  # planeSweeping
  cuda/planeSweeping/device_code.cu
  cuda/planeSweeping/device_code_refine.cu
  cuda/planeSweeping/device_code_volume.cu
  cuda/planeSweeping/device_code_fuse.cu
  cuda/planeSweeping/device_utils.cu
  cuda/planeSweeping/device_utils.h
)

set_source_files_properties(${depthMap_cuda_files_headers}
  PROPERTIES HEADER_FILE_ONLY true
)

# Cuda Sources
set(depthMap_cuda_files_sources
  cuda/commonStructures.hpp
  cuda/PlaneSweepingCuda.cpp
  cuda/PlaneSweepingCuda.hpp
  cuda/planeSweeping/plane_sweeping_cuda.cu
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
    ${Boost_FILESYSTEM_LIBRARY}
    ${CUDA_CUDADEVRT_LIBRARY}
    ${CUDA_CUBLAS_LIBRARIES} #TODO shouldn't be here, but required to build on some machines
  PRIVATE_LINKS
    aliceVision_gpu
    aliceVision_sfmData
    aliceVision_sfmDataIO
  PUBLIC_INCLUDE_DIRS
    ${CUDA_INCLUDE_DIRS}
)
