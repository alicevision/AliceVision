add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_samples_robustHomographyGrowing main_robustHomographyGrowing.cpp)

target_link_libraries(aliceVision_samples_robustHomographyGrowing
  aliceVision_system
  aliceVision_image
  aliceVision_feature
  aliceVision_matching
  aliceVision_matchingImageCollection
  vlsift
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_samples_robustHomographyGrowing
  PROPERTY FOLDER AliceVision/Samples
)
