
# Headers
set(image_processing_file_headers
    imageProcessing.hpp
)

# Sources
set(image_processing_file_sources
    imageProcessing.cpp
)

if (ALICEVISION_HAVE_OPENCV)
    list(APPEND image_processing_file_headers imageProcessing_OpenCV.hpp)
    list(APPEND image_processing_file_sources imageProcessing_OpenCV.cpp)
endif()

alicevision_add_library(aliceVision_imageProcessing
    SOURCES ${image_processing_file_headers} ${image_processing_file_sources}
    PUBLIC_LINKS
        aliceVision_numeric
        aliceVision_image
        aliceVision_sfmData
        aliceVision_camera
        aliceVision_lensCorrectionProfile
    PRIVATE_LINKS
        aliceVision_system
)

if (ALICEVISION_HAVE_OPENCV)
    target_link_libraries(aliceVision_imageProcessing PUBLIC ${OpenCV_LIBS})
endif()