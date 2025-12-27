# Headers
set(depthMap_files_headers
    BufPtr.hpp
    computeOnMultiDevices.hpp
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
    computeOnMultiDevices.cpp
    CustomPatchPatternParams.cpp
    DepthMapEstimator.cpp
    depthMapUtils.cpp
    NormalMapEstimator.cpp
    Refine.cpp
    Sgm.cpp
    SgmDepthList.cpp
    volumeIO.cpp
)

alicevision_add_library(aliceVision_depthMap
    USE_SYCL
    SOURCES
        ${depthMap_files_headers}
        ${depthMap_files_sources}
    PUBLIC_LINKS
        aliceVision_mvsData
        aliceVision_mvsUtils
        aliceVision_system
        assimp::assimp
    PRIVATE_LINKS
        aliceVision_sfmData
        aliceVision_sfmDataIO
    PUBLIC_INCLUDE_DIRS
        ${SYCL_INCLUDE_DIRS}
)

