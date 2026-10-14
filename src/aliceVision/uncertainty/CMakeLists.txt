# Headers
set(uncertainty_files_headers
    uncertainty_kernels.hpp
)

# Sources
set(uncertainty_files_sources
    uncertainty.cpp
    uncertainty_kernels.cu
)

alicevision_add_library(aliceVision_uncertainty
    USE_CUDA
    SOURCES ${uncertainty_files_headers} ${uncertainty_files_sources}
    PUBLIC_LINKS
        aliceVision_sfmData
        ${MAGMA_LIBRARIES}
        CUDA::cudart
    PRIVATE_LINKS
        aliceVision_system
)

# Unit tests
alicevision_add_test(uncertainty_kernels_test.cpp
    NAME "uncertainty_kernels"
    LINKS aliceVision_uncertainty CUDA::cudart
)

alicevision_add_test(uncertainty_test.cpp
    NAME "uncertainty"
    LINKS aliceVision_uncertainty
          aliceVision_sfm
          aliceVision_sfmDataIO
          aliceVision_sfmData
          aliceVision_feature
          aliceVision_system
          CUDA::cudart
)