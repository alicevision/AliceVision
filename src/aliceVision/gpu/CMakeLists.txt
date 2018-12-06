# Headers
set(gpu_files_headers
  gpu.hpp
)

# Sources
set(gpu_files_sources
  gpu.cpp
)

set(GPU_PRIVATE_LINKS aliceVision_system)
set(GPU_PRIVATE_INCLUDE_DIRS "")

if(ALICEVISION_HAVE_CUDA)
  set(GPU_PRIVATE_LINKS ${GPU_PRIVATE_LINKS} ${CUDA_LIBRARIES})
  set(GPU_PRIVATE_INCLUDE_DIRS ${GPU_PRIVATE_INCLUDE_DIRS} ${CUDA_INCLUDE_DIRS})
endif()

alicevision_add_library(aliceVision_gpu
  SOURCES ${gpu_files_headers} ${gpu_files_sources}
  PRIVATE_LINKS
    ${GPU_PRIVATE_LINKS}
  PRIVATE_INCLUDE_DIRS
    ${GPU_PRIVATE_INCLUDE_DIRS}
)