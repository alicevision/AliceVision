
ADD_DEFINITIONS(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

ADD_EXECUTABLE(aliceVision_sample_akaze akaze_demo.cpp)
TARGET_LINK_LIBRARIES(aliceVision_sample_akaze
  aliceVision_image
  aliceVision_features
  stlplus
  ${ALICEVISION_LIBRARY_DEPENDENCIES})
SET_PROPERTY(TARGET aliceVision_sample_akaze PROPERTY FOLDER AliceVision/Samples)

ADD_EXECUTABLE( aliceVision_main_computeAKAZE main_computeAKAZE.cpp )
TARGET_LINK_LIBRARIES( aliceVision_main_computeAKAZE
  aliceVision_image
  aliceVision_features
  aliceVision_system
  stlplus
  ${ALICEVISION_LIBRARY_DEPENDENCIES})
SET_PROPERTY(TARGET aliceVision_main_computeAKAZE PROPERTY FOLDER AliceVision/Samples)

