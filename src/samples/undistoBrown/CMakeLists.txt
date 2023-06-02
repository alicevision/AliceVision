alicevision_add_software(aliceVision_samples_undistoBrown
  SOURCE main_undistoBrown.cpp
  FOLDER ${FOLDER_SAMPLES}
  LINKS aliceVision_cmdline
        aliceVision_system
        aliceVision_image
        aliceVision_multiview
        aliceVision_camera
        ${CERES_LIBRARIES}
        Boost::program_options
        Boost::filesystem
        Boost::regex
        Boost::boost
)
