if(ALICEVISION_HAVE_OPENCV)
  add_definitions(-DTHIS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
  alicevision_add_software(alicevision_samples_openCVFitting
    SOURCE main_openCVFitting.cpp
    FOLDER ${FOLDER_SAMPLES}
    LINKS aliceVision_multiview
          ${OpenCV_LIBS}
          Boost::boost
          Boost::container
  )
endif()


