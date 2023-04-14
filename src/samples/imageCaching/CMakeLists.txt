alicevision_add_software(aliceVision_samples_imageCaching
  SOURCE main_imageCaching.cpp
  FOLDER ${FOLDER_SAMPLES}
  LINKS aliceVision_system
        aliceVision_image
        aliceVision_cmdline
        Boost::program_options
)
