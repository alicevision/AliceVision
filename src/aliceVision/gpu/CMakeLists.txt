# Headers
set(gpu_files_headers
    gpu.hpp
)

# Sources
set(gpu_files_sources
    gpu.cpp
)

set(GPU_PRIVATE_LINKS aliceVision_system)

if (ALICEVISION_HAVE_CUDA)
    set(GPU_PRIVATE_LINKS ${GPU_PRIVATE_LINKS} ${ALICEVISION_CUDA_LIBRARIES})
endif()

alicevision_add_library(aliceVision_gpu
    SOURCES ${gpu_files_headers} ${gpu_files_sources}
    PRIVATE_LINKS
        ${GPU_PRIVATE_LINKS}
)