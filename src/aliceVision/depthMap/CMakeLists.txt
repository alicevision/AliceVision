# Headers
set(depthMap_files_headers
  BufPtr.hpp
  computeOnMultiGPUs.hpp
  CustomPatchPatternParams.hpp
  DepthMapEstimator.hpp
  DepthMapParams.hpp
  depthMapUtils.hpp
  NormalMapEstimator.hpp
  Refine.hpp
  RefineParams.hpp
  Sgm.hpp
  SgmDepthList.hpp
  SgmParams.hpp
  Tile.hpp
  volumeIO.hpp
)

# Sources
set(depthMap_files_sources
  computeOnMultiGPUs.cpp
  CustomPatchPatternParams.cpp
  DepthMapEstimator.cpp
  depthMapUtils.cpp
  NormalMapEstimator.cpp
  Refine.cpp
  Sgm.cpp
  SgmDepthList.cpp
  volumeIO.cpp
)

# Cuda Host Headers Only
set(depthMap_cuda_host_headers
  cuda/host/LRUCameraCache.hpp
  cuda/host/LRUCache.hpp
  cuda/host/divUp.hpp
  cuda/host/memory.hpp
)

# Cuda Host Sources
set(depthMap_cuda_host_sources
  cuda/host/DeviceCache.hpp
  cuda/host/DeviceCache.cpp
  cuda/host/DeviceMipmapImage.hpp
  cuda/host/DeviceMipmapImage.cpp
  cuda/host/DeviceStreamManager.hpp
  cuda/host/DeviceStreamManager.cpp
  cuda/host/patchPattern.hpp
  cuda/host/patchPattern.cpp
  cuda/host/utils.hpp
  cuda/host/utils.cpp
)

# device CUDA Headers Only
set(depthMap_cuda_device_headers
  cuda/device/buffer.cuh
  cuda/device/color.cuh
  cuda/device/eig33.cuh
  cuda/device/matrix.cuh
  cuda/device/operators.cuh
  cuda/device/Patch.cuh
  cuda/device/SimStat.cuh
)

# device CUDA Sources
set(depthMap_cuda_device_sources
  cuda/device/DeviceCameraParams.hpp
  cuda/device/DeviceCameraParams.cu
  cuda/device/DevicePatchPattern.hpp
  cuda/device/DevicePatchPattern.cu
)

# imageProcessing CUDA Sources
set(depthMap_cuda_imageProcessing_sources
  cuda/imageProcessing/deviceGaussianFilter.hpp
  cuda/imageProcessing/deviceGaussianFilter.cu
  cuda/imageProcessing/deviceColorConversion.hpp
  cuda/imageProcessing/deviceColorConversion.cu
  cuda/imageProcessing/deviceMipmappedArray.hpp
  cuda/imageProcessing/deviceMipmappedArray.cu
)

# planeSweeping CUDA Headers Only
set(depthMap_cuda_planeSweeping_headers
  cuda/planeSweeping/deviceDepthSimilarityMapKernels.cuh
  cuda/planeSweeping/deviceSimilarityVolumeKernels.cuh
)

# planeSweeping CUDA Sources
set(depthMap_cuda_planeSweeping_sources
  cuda/planeSweeping/similarity.hpp
  cuda/planeSweeping/deviceDepthSimilarityMap.hpp
  cuda/planeSweeping/deviceDepthSimilarityMap.cu
  cuda/planeSweeping/deviceSimilarityVolume.hpp
  cuda/planeSweeping/deviceSimilarityVolume.cu
)

set_source_files_properties(${depthMap_cuda_host_headers}
			    ${depthMap_cuda_device_headers} 
			    ${depthMap_cuda_planeSweeping_headers}

  PROPERTIES HEADER_FILE_ONLY true
)

source_group("aliceVision_depthMap_cuda_host" FILES ${depthMap_cuda_host_headers} ${depthMap_cuda_host_sources})
source_group("aliceVision_depthMap_cuda_device" FILES ${depthMap_cuda_device_headers} ${depthMap_cuda_device_sources})
source_group("aliceVision_depthMap_cuda_imageProcessing" FILES ${depthMap_cuda_imageProcessing_sources})
source_group("aliceVision_depthMap_cuda_planeSweeping" FILES ${depthMap_cuda_planeSweeping_headers} ${depthMap_cuda_planeSweeping_sources})

# Cuda Sources
set(depthMap_cuda_files_sources
  ${depthMap_cuda_host_headers} 
  ${depthMap_cuda_host_sources}
  ${depthMap_cuda_device_headers} 
  ${depthMap_cuda_device_sources}
  ${depthMap_cuda_imageProcessing_sources}
  ${depthMap_cuda_planeSweeping_headers} 
  ${depthMap_cuda_planeSweeping_sources}
)

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
    assimp::assimp
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

