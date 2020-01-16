# Multi-Band Blending
alicevision_add_software(aliceVision_samples_multibandtest
  SOURCE main_multibandtest.cpp
  FOLDER ${FOLDER_SAMPLES}
  LINKS aliceVision_system
        aliceVision_mesh
        aliceVision_mvsData
        Boost::program_options
)
