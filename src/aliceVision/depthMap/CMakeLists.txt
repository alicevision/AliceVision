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

# Files excluded from compilation
set(depthMap_cuda_files_headers
  # Cuda
  cuda/planeSweeping/device_code.cu
  cuda/planeSweeping/device_code_refine.cu
  cuda/planeSweeping/device_code_volume.cu
  cuda/planeSweeping/device_code_fuse.cu
  cuda/planeSweeping/device_utils.cu
)

set(depthMap_cuda_deviceCommon_files_headers
  # Headers
  cuda/deviceCommon/device_patch_es_glob.hpp
  # Cuda
  cuda/deviceCommon/device_color.cu
  cuda/deviceCommon/device_eig33.cu
  cuda/deviceCommon/device_global.cu
  cuda/deviceCommon/device_matrix.cu
  cuda/deviceCommon/device_patch_es.cu
  cuda/deviceCommon/device_simStat.cu
)

set_source_files_properties(${depthMap_cuda_files_headers}
  PROPERTIES HEADER_FILE_ONLY true
)

set_source_files_properties(${depthMap_cuda_deviceCommon_files_headers}
  PROPERTIES HEADER_FILE_ONLY true
)

# Cuda Sources

set(depthMap_cuda_files_sources
  cuda/commonStructures.hpp
  cuda/PlaneSweepingCuda.cpp
  cuda/PlaneSweepingCuda.hpp
  cuda/planeSweeping/plane_sweeping_cuda.cu
  ${depthMap_cuda_files_headers}
  ${depthMap_cuda_deviceCommon_files_headers}
)
source_group("depthMap_cuda" FILES ${depthMap_cuda_files_sources})

if(BUILD_SHARED_LIBS)
  cuda_add_library(aliceVision_depthMap
    SHARED ${depthMap_files_headers}
           ${depthMap_files_sources}
           ${depthMap_cuda_files_sources}
    OPTIONS --compiler-options "-fPIC"
  )
else()
  cuda_add_library(aliceVision_depthMap
    ${depthMap_files_headers}
    ${depthMap_files_sources}
    ${depthMap_cuda_files_sources}
  )
endif()

target_include_directories(aliceVision_depthMap
  PUBLIC $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
         $<BUILD_INTERFACE:${generatedDir}>
         $<INSTALL_INTERFACE:include>
         ${CUDA_INCLUDE_DIRS}
)

# TODO : PUBLIC
target_link_libraries(aliceVision_depthMap
  aliceVision_mvsData
  aliceVision_imageIO
  aliceVision_mvsUtils
  aliceVision_system
  ${Boost_FILESYSTEM_LIBRARY}
  ${CUDA_CUDADEVRT_LIBRARY}
  ${CUDA_CUBLAS_LIBRARIES}
)

set_property(TARGET aliceVision_depthMap
  PROPERTY FOLDER AliceVision
)

install(TARGETS aliceVision_depthMap
  DESTINATION lib
  EXPORT aliceVision-targets
)
