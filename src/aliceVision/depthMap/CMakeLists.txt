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

# Cuda Common Headers
set(depthMap_cuda_common_headers
  cuda/hostUtils.hpp
)

# Cuda Common Sources
set(depthMap_cuda_common_sources
  cuda/memory.hpp
  cuda/utils.hpp
  cuda/utils.cpp
  cuda/DeviceStreamManager.cpp
  cuda/DeviceStreamManager.hpp
  cuda/DeviceCache.cpp
  cuda/DeviceCache.hpp
  cuda/DeviceCamera.cpp
  cuda/DeviceCamera.hpp
  cuda/LRUCameraCache.hpp
  cuda/LRUCache.hpp
  cuda/ROI.hpp
  cuda/PlaneSweepingCuda.cpp
  cuda/PlaneSweepingCuda.hpp
)

# device CUDA Headers
set(depthMap_cuda_device_headers
  cuda/device/color.cuh
  cuda/device/matrix.cuh
  cuda/device/operators.cuh
  cuda/device/utils.cuh
  cuda/device/Patch.cuh
  cuda/device/SimStat.cuh
  cuda/device/device_utils.h
)

# device CUDA Sources
set(depthMap_cuda_device_sources
  cuda/device/DeviceCameraParams.hpp
  cuda/device/DeviceCameraParams.cu
)

# imageProcessing CUDA Sources
set(depthMap_cuda_imageProcessing_sources
  cuda/imageProcessing/deviceGaussianFilter.hpp
  cuda/imageProcessing/deviceGaussianFilter.cu
  cuda/imageProcessing/deviceColorConversion.hpp
  cuda/imageProcessing/deviceColorConversion.cu
)

# normalMapping CUDA Headers
set(depthMap_cuda_normalMapping_headers
  cuda/normalMapping/eig33.cuh
)

# normalMapping CUDA Sources
set(depthMap_cuda_normalMapping_sources
  cuda/normalMapping/deviceNormalMap.hpp
  cuda/normalMapping/deviceNormalMap.cu
  cuda/normalMapping/DeviceNormalMapper.hpp
  cuda/normalMapping/DeviceNormalMapper.cpp
)

# planeSweeping CUDA Headers
set(depthMap_cuda_planeSweeping_headers
  cuda/planeSweeping/deviceFuseKernels.cuh
  cuda/planeSweeping/deviceRefineKernels.cuh
  cuda/planeSweeping/deviceSimilarityVolumeKernels.cuh
)

# planeSweeping CUDA Sources
set(depthMap_cuda_planeSweeping_sources
  cuda/planeSweeping/similarity.hpp
  cuda/planeSweeping/deviceFuse.hpp
  cuda/planeSweeping/deviceFuse.cu
  cuda/planeSweeping/deviceRefine.hpp
  cuda/planeSweeping/deviceRefine.cu
  cuda/planeSweeping/deviceSimilarityVolume.hpp
  cuda/planeSweeping/deviceSimilarityVolume.cu
)

set_source_files_properties(${depthMap_cuda_common_headers}
		            ${depthMap_cuda_device_headers} 
			    ${depthMap_cuda_normalMapping_headers}
			    ${depthMap_cuda_planeSweeping_headers}

  PROPERTIES HEADER_FILE_ONLY true
)

source_group("aliceVision_depthMap_cuda_common" FILES ${depthMap_cuda_common_headers} ${depthMap_cuda_common_sources})
source_group("aliceVision_depthMap_cuda_device" FILES ${depthMap_cuda_device_headers} ${depthMap_cuda_device_sources})
source_group("aliceVision_depthMap_cuda_imageProcessing" FILES ${depthMap_cuda_imageProcessing_sources})
source_group("aliceVision_depthMap_cuda_normalMapping" FILES ${depthMap_cuda_normalMapping_headers} ${depthMap_cuda_normalMapping_sources})
source_group("aliceVision_depthMap_cuda_planeSweeping" FILES ${depthMap_cuda_planeSweeping_headers} ${depthMap_cuda_planeSweeping_sources})

# Cuda Sources
set(depthMap_cuda_files_sources
  ${depthMap_cuda_common_headers} 
  ${depthMap_cuda_common_sources}
  ${depthMap_cuda_device_headers} 
  ${depthMap_cuda_device_sources}
  ${depthMap_cuda_imageProcessing_sources}
  ${depthMap_cuda_normalMapping_headers} 
  ${depthMap_cuda_normalMapping_sources}
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

