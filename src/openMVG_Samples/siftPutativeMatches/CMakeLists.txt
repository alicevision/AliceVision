
ADD_DEFINITIONS(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

LINK_DIRECTORIES("/local/home/griff/InstallWorking/lib") #ugly hack

ADD_EXECUTABLE(openMVG_sample_siftPutative siftmatch.cpp)
TARGET_LINK_LIBRARIES(openMVG_sample_siftPutative
  openMVG_image
  openMVG_features
  openMVG_matching
  vlsift
  popsift
  ${CUDA_LIBRARIES}
  ${CUDA_CUDADEVRT_LIBRARY}
  stlplus)

SET_PROPERTY(TARGET openMVG_sample_siftPutative PROPERTY FOLDER OpenMVG/Samples)
