alicevision_add_software(aliceVision_samples_parseDatabase
  SOURCE main_parseDatabase.cpp
  FOLDER ${FOLDER_SAMPLES}
  LINKS aliceVision_system
        aliceVision_sensorDB
        Boost::program_options
)
