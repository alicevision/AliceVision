find_package(PopSift)

ADD_DEFINITIONS(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

ADD_EXECUTABLE(openMVG_sample_siftPutative siftmatch.cpp)
TARGET_LINK_LIBRARIES(openMVG_sample_siftPutative PUBLIC
  openMVG_image
  openMVG_features
  openMVG_matching
  vlsift
  PopSift::popsift
  ${CUDA_LIBRARIES}
  ${CUDA_CUDADEVRT_LIBRARY}
  ${OpenCV_LIBS}
  stlplus
  )
TARGET_INCLUDE_DIRECTORIES(openMVG_sample_siftPutative PUBLIC
  ${POPSIFT_INCLUDE_DIRS}
  )

SET_PROPERTY(TARGET openMVG_sample_siftPutative PROPERTY FOLDER OpenMVG/Samples)
