## AliceVision
## Convertion software

# Software PROPERTY FOLDER is 'Software/Convert'
set(FOLDER_SOFTWARE_CONVERT "Software/Convert")

if(ALICEVISION_BUILD_SFM)
    # Convert SfMData format (from one to another)
    alicevision_add_software(aliceVision_convertSfMFormat
        SOURCE main_convertSfMFormat.cpp
        FOLDER ${FOLDER_SOFTWARE_CONVERT}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_feature
              aliceVision_sfmData
              aliceVision_sfmDataIO
              Boost::program_options
              Boost::system
    )

    alicevision_add_software(aliceVision_importKnownPoses
        SOURCE main_importKnownPoses.cpp
        FOLDER ${FOLDER_SOFTWARE_CONVERT}
        LINKS aliceVision_localization
              aliceVision_feature
              aliceVision_sfmData
              aliceVision_sfmDataIO
              aliceVision_cmdline
              Boost::program_options
              Boost::boost
              Boost::timer
    )

    if (TARGET E57Format)
        alicevision_add_software(aliceVision_importE57
            SOURCE main_importE57.cpp
            FOLDER ${FOLDER_SOFTWARE_CONVERT}
            LINKS 
                aliceVision_system
                aliceVision_cmdline
                aliceVision_feature
                aliceVision_sfmData
                aliceVision_sfmDataIO
                aliceVision_mesh
                aliceVision_fuseCut
                Boost::program_options
                Boost::json
                E57Format
                nanoflann
        )
    endif()

    # Convert Distortion format (from one to another)
    alicevision_add_software(aliceVision_convertDistortion
        SOURCE main_convertDistortion.cpp
        FOLDER ${FOLDER_SOFTWARE_CONVERT}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_feature
              aliceVision_sfmData
              aliceVision_sfmDataIO
              aliceVision_calibration
              Boost::program_options
              Boost::system
    )
    
endif() # ALICEVISION_BUILD_SFM


if(ALICEVISION_BUILD_MVS)
    # Merge two meshes
    alicevision_add_software(aliceVision_convertMesh
        SOURCE main_convertMesh.cpp
        FOLDER ${FOLDER_SOFTWARE_CONVERT}
        LINKS aliceVision_system
              aliceVision_numeric
              aliceVision_cmdline
              aliceVision_mesh
              ${Boost_LIBRARIES}
    )
endif() # ALICEVISION_BUILD_MVS
