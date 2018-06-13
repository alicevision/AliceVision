add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(aliceVision_samples_kvldFilter main_kvldFilter.cpp)

target_link_libraries(aliceVision_samples_kvldFilter
  aliceVision_system
  aliceVision_image
  aliceVision_multiview
  aliceVision_kvld
  aliceVision_feature
  aliceVision_matching
  vlsift
  ${Boost_LIBRARIES}
)

set_property(TARGET aliceVision_samples_kvldFilter
  PROPERTY FOLDER Samples
)
