# Headers
set(intrinsicDecomposition_files_headers
  CudaArray.hpp
  NamedParameters.hpp
  OptImage.hpp
  OptUtils.hpp
  SolverIteration.hpp
  cudaUtil.hpp
  CombinedSolverBase.hpp
  IntrinsicDecompositionSolver.hpp
  OptGraph.hpp
  OptSolver.hpp
  SolverBase.hpp
  SolverParams.hpp
  intrinsicImageDecomposition.opt
  )

# Sources
set(intrinsicDecomposition_files_sources
  IntrinsicDecompositionSolver.cpp
  SolverIteration.cpp
  )

get_filename_component(TERRA_LIB_PATH "$ENV{OPTLANG_LIBDIR}/libterra.so" ABSOLUTE)
get_filename_component(OPT_LIB_PATH "$ENV{OPTLANG_LIBDIR}/libOpt.a" ABSOLUTE)
get_filename_component(OPTDEV_LIB_PATH "$ENV{OPTLANG_LIBDIR}/libOptDev.a" ABSOLUTE)
get_filename_component(OPT_INCLUDE_PATH "$ENV{OPTLANG_ROOT}/include" ABSOLUTE)


message(WARNING "TERRA_LIB_PATH: ${TERRA_LIB_PATH}")
message(WARNING "OPT_LIB_PATH: ${OPT_LIB_PATH}")
message(WARNING "OPTDEV_LIB_PATH: ${OPTDEV_LIB_PATH}")
message(WARNING "CUDA_CUDADEVRT_LIBRARY: ${CUDA_CUDADEVRT_LIBRARY}")
message(WARNING "CUDA_CUDART_LIBRARY: ${CUDA_CUDART_LIBRARY}")
message(WARNING "OPT_INCLUDE_PATH: ${OPT_INCLUDE_PATH}")

alicevision_add_library(aliceVision_intrinsicDecomposition
  SOURCES ${intrinsicDecomposition_files_headers} ${intrinsicDecomposition_files_sources}
  PUBLIC_LINKS
    aliceVision_imageIO
    aliceVision_sfmData
    aliceVision_sfmDataIO
    ${Boost_LIBRARIES}
    ${CUDA_CUDADEVRT_LIBRARY}
    ${CUDA_CUDART_LIBRARY}
    ${TERRA_LIB_PATH}
    ${OPT_LIB_PATH}
    ${OPTDEV_LIB_PATH}
  PUBLIC_INCLUDE_DIRS
    ${CUDA_INCLUDE_DIRS}
    ${OPT_INCLUDE_PATH}
  )


# Unit tests
# alicevision_add_test(albedoEstimation_test.cpp
#   NAME "albedoEstimation_albedoEstimation"
#   LINKS aliceVision_albedoEstimation
# )
