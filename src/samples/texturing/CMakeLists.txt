alicevision_add_software(aliceVision_samples_evcorrection 
  SOURCE main_evcorrection.cpp 
  FOLDER ${FOLDER_SAMPLES} 
  LINKS aliceVision_system 
        aliceVision_sfmData 
        aliceVision_image 
        ${Boost_LIBRARIES} 
        ${OPENIMAGEIO_LIBRARIES} 
)