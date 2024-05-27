## AliceVision
## Pipeline softwares

# Software PROPERTY FOLDER is 'Software/Pipeline'
set(FOLDER_SOFTWARE_PIPELINE "Software/Pipeline")

### SfM software
if(ALICEVISION_BUILD_SFM)

    # Intrinsic image analysis and SfMData container initialization
    alicevision_add_software(aliceVision_cameraInit
        SOURCE main_cameraInit.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_sensorDB
              aliceVision_image
              aliceVision_feature
              aliceVision_sfmData
              aliceVision_sfmDataIO
              aliceVision_lensCorrectionProfile
              Boost::program_options
              Boost::boost
    )

    # Image processing
    if(ALICEVISION_HAVE_OPENCV)
        alicevision_add_software(aliceVision_imageMasking
            SOURCE main_imageMasking.cpp
            FOLDER ${FOLDER_SOFTWARE_PIPELINE}
            LINKS aliceVision_image
                  aliceVision_imageMasking
                  aliceVision_system
                  aliceVision_cmdline
                  aliceVision_sfmData
                  aliceVision_sfmDataIO
                  ${OpenCV_LIBRARIES}
                  ${Boost_LIBRARIES}
        )
    endif()

    # Feature extraction
    alicevision_add_software(aliceVision_featureExtraction
        SOURCE main_featureExtraction.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_gpu
              aliceVision_image
              aliceVision_feature
              aliceVision_featureEngine
              aliceVision_multiview
              aliceVision_sfmData
              aliceVision_sfmDataIO
              vlsift
              Boost::program_options
              Boost::boost
              Boost::timer
    )

    # Image matching
    # - generate the image pair lists
    alicevision_add_software(aliceVision_imageMatching
        SOURCE main_imageMatching.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_imageMatching
              aliceVision_matchingImageCollection
              aliceVision_sfm
              aliceVision_sfmData
              aliceVision_sfmDataIO
              aliceVision_voctree
              Boost::program_options
    )

    # Feature matching
    # - putative matches and geometric filtered matches
    alicevision_add_software(aliceVision_featureMatching
        SOURCE main_featureMatching.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_feature
              aliceVision_multiview
              aliceVision_matchingImageCollection
              aliceVision_sfm
              aliceVision_sfmData
              aliceVision_sfmDataIO
              Boost::program_options
    )

    # Track builder
    alicevision_add_software(aliceVision_tracksBuilding
        SOURCE main_tracksBuilding.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_feature
              aliceVision_sfm
              aliceVision_sfmData
              aliceVision_track
              Boost::program_options
              Boost::json
    )

    # Relative pose
    alicevision_add_software(aliceVision_relativePoseEstimating
        SOURCE main_relativePoseEstimating.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_feature
              aliceVision_sfm
              aliceVision_sfmData
              aliceVision_track
              Boost::program_options
              Boost::json
    )

    # Bootstraping SfM
    alicevision_add_software(aliceVision_sfmBootstraping
        SOURCE main_sfmBootstraping.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_feature
              aliceVision_sfm
              aliceVision_sfmData
              aliceVision_track
              aliceVision_dataio
              Boost::program_options
              Boost::json
    )

    # Incremental / Sequential SfM
    alicevision_add_software(aliceVision_incrementalSfM
        SOURCE main_incrementalSfM.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_image
              aliceVision_feature
              aliceVision_sfm
              aliceVision_sfmData
              aliceVision_sfmDataIO
              Boost::program_options
    )

    # Incremental SFM for pure rotation
    alicevision_add_software(aliceVision_nodalSfM
        SOURCE main_nodalSfM.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_image
              aliceVision_feature
              aliceVision_sfm
              aliceVision_sfmData
              aliceVision_sfmDataIO
              Boost::program_options
    )

    # SfM Triangulation
    alicevision_add_software(aliceVision_sfmTriangulation
        SOURCE main_sfmTriangulation.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_image
              aliceVision_feature
              aliceVision_sfm
              aliceVision_sfmData
              aliceVision_sfmDataIO
              Boost::program_options
    )

    # Global SfM
    alicevision_add_software(aliceVision_globalSfM
        SOURCE main_globalSfM.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_image
              aliceVision_feature
              aliceVision_sfm
              aliceVision_sfmData
              aliceVision_sfmDataIO
              Boost::program_options
    )

    # Compute structure from known camera poses
    alicevision_add_software(aliceVision_computeStructureFromKnownPoses
        SOURCE main_computeStructureFromKnownPoses.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_feature
              aliceVision_sfm
              aliceVision_sfmData
              aliceVision_sfmDataIO
              Boost::program_options
    )

    # Calibrate a camera
    if(ALICEVISION_HAVE_OPENCV)
        alicevision_add_software(aliceVision_cameraCalibration
            SOURCE main_cameraCalibration.cpp
            FOLDER ${FOLDER_SOFTWARE_PIPELINE}
            LINKS aliceVision_dataio
                  aliceVision_image
                  aliceVision_calibration
                  aliceVision_system
                  aliceVision_cmdline
                  Boost::program_options
                  Boost::boost
        )
    endif()

    alicevision_add_software(aliceVision_distortionCalibration
        SOURCE main_distortionCalibration.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_dataio
              aliceVision_image
              aliceVision_calibration
              aliceVision_system
              aliceVision_cmdline
              aliceVision_sfmDataIO
              Boost::program_options
              Boost::boost
              Boost::json
    )

    alicevision_add_software(aliceVision_checkerboardCalibration
        SOURCE main_checkerboardCalibration.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_dataio
              aliceVision_image
              aliceVision_calibration
              aliceVision_system
              aliceVision_cmdline
              aliceVision_sfmDataIO
              aliceVision_multiview
              aliceVision_sfm
              Boost::program_options
              Boost::json
              Boost::boost
    )

    # Calibrate a rig
    alicevision_add_software(aliceVision_rigCalibration
        SOURCE main_rigCalibration.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_rig
              aliceVision_localization
              aliceVision_dataio
              aliceVision_image
              aliceVision_feature
              aliceVision_cmdline
              Boost::program_options
              Boost::boost
              Boost::timer
    )

    alicevision_add_software(aliceVision_checkerboardDetection
        SOURCE main_checkerboardDetection.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_dataio
              aliceVision_image
              aliceVision_calibration
              aliceVision_system
              aliceVision_sfmDataIO
              aliceVision_cmdline
              Boost::program_options
              Boost::boost
              Boost::json
    )

    if(ALICEVISION_HAVE_CCTAG)
        target_link_libraries(aliceVision_rigCalibration_exe PUBLIC CCTag::CCTag)
    endif()

    # Localize a camera
    alicevision_add_software(aliceVision_cameraLocalization
        SOURCE main_cameraLocalization.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_localization
              aliceVision_dataio
              aliceVision_image
              aliceVision_feature
              aliceVision_cmdline
              Boost::program_options
              Boost::boost
              Boost::timer
    )

    if(ALICEVISION_HAVE_CCTAG)
        target_link_libraries(aliceVision_cameraLocalization_exe PUBLIC CCTag::CCTag)
    endif()

    # Localize a rig
    alicevision_add_software(aliceVision_rigLocalization
        SOURCE main_rigLocalization.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_localization
              aliceVision_dataio
              aliceVision_rig
              aliceVision_image
              aliceVision_feature
              aliceVision_cmdline
              Boost::program_options
              Boost::boost
              Boost::timer
    )

    if(ALICEVISION_HAVE_CCTAG)
        target_link_libraries(aliceVision_rigLocalization_exe PUBLIC CCTag::CCTag)
    endif()

    # Prepare dense scene for MVS
    alicevision_add_software(aliceVision_prepareDenseScene
        SOURCE main_prepareDenseScene.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_image
              aliceVision_feature
              aliceVision_sfmData
              aliceVision_sfmDataIO
              Boost::program_options
              Boost::boost
              Boost::timer
    )
         
endif() # if(ALICEVISION_BUILD_SFM)

if(ALICEVISION_BUILD_PANORAMA)
    # Panorama
    alicevision_add_software(aliceVision_panoramaPrepareImages
        SOURCE main_panoramaPrepareImages.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_sensorDB
              aliceVision_image
              aliceVision_feature
              aliceVision_sfmData
              aliceVision_sfmDataIO
              Boost::program_options
              Boost::boost
    )

    alicevision_add_software(aliceVision_panoramaEstimation
        SOURCE main_panoramaEstimation.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_image
              aliceVision_feature
              aliceVision_sfm
              aliceVision_sfmData
              aliceVision_sfmDataIO
              ${Boost_LIBRARIES}
    )
    alicevision_add_software(aliceVision_panoramaWarping
        SOURCE main_panoramaWarping.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_image
              aliceVision_feature
              aliceVision_sfm
              aliceVision_sfmData
              aliceVision_sfmDataIO
              aliceVision_panorama
              ${Boost_LIBRARIES}
    )
    alicevision_add_software(aliceVision_panoramaMerging
        SOURCE main_panoramaMerging.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_image
              aliceVision_sfmData
              aliceVision_sfmDataIO
              aliceVision_panorama
              ${Boost_LIBRARIES}
    )
    alicevision_add_software(aliceVision_panoramaPostProcessing
        SOURCE main_panoramaPostProcessing.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_image
              ${Boost_LIBRARIES}
    )
    alicevision_add_software(aliceVision_panoramaCompositing
        SOURCE main_panoramaCompositing.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_image
              aliceVision_sfmData
              aliceVision_sfmDataIO
              aliceVision_panorama
              ${Boost_LIBRARIES}
    )
    alicevision_add_software(aliceVision_panoramaSeams
        SOURCE main_panoramaSeams.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_image
              aliceVision_sfmData
              aliceVision_sfmDataIO
              aliceVision_panorama
              ${Boost_LIBRARIES}
    )
    alicevision_add_software(aliceVision_panoramaInit
        SOURCE main_panoramaInit.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_image
              aliceVision_feature
              aliceVision_sfm
              aliceVision_sfmData
              aliceVision_sfmDataIO
              ${Boost_LIBRARIES}
    )
endif()

### MVS software
if(ALICEVISION_BUILD_MVS)

    if(ALICEVISION_HAVE_CUDA) # Depth map computation need CUDA
        # Depth Map Estimation
        alicevision_add_software(aliceVision_depthMapEstimation
            SOURCE main_depthMapEstimation.cpp
            FOLDER ${FOLDER_SOFTWARE_PIPELINE}
            LINKS aliceVision_system
                  aliceVision_cmdline
                  aliceVision_gpu
                  aliceVision_mvsData
                  aliceVision_mvsUtils
                  aliceVision_depthMap
                  aliceVision_sfmData
                  aliceVision_sfmDataIO
                  Boost::program_options
        )

        # Depth Map Filtering
        alicevision_add_software(aliceVision_depthMapFiltering
            SOURCE main_depthMapFiltering.cpp
            FOLDER ${FOLDER_SOFTWARE_PIPELINE}
            LINKS aliceVision_system
                  aliceVision_cmdline
                  aliceVision_mvsData
                  aliceVision_mvsUtils
                  aliceVision_fuseCut
                  aliceVision_depthMap
                  aliceVision_sfmData
                  aliceVision_sfmDataIO
                  Boost::program_options
        )
    endif()

    # Meshing
    alicevision_add_software(aliceVision_meshing
        SOURCE main_meshing.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_mvsData
              aliceVision_mvsUtils
              aliceVision_mesh
              aliceVision_fuseCut
              aliceVision_sfmData
              aliceVision_sfmDataIO
              Boost::program_options
    )

    # Lidar meshing
    alicevision_add_software(aliceVision_lidarMeshing
        SOURCE main_lidarMeshing.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_mvsData
              aliceVision_mvsUtils
              aliceVision_mesh
              aliceVision_fuseCut
              aliceVision_sfmData
              aliceVision_sfmDataIO
              Boost::program_options
              Boost::json
    )

     # Lidar merging
     alicevision_add_software(aliceVision_lidarMerging
     SOURCE main_lidarMerging.cpp
     FOLDER ${FOLDER_SOFTWARE_PIPELINE}
     LINKS aliceVision_system
           aliceVision_cmdline
           aliceVision_mvsData
           aliceVision_mvsUtils
           aliceVision_mesh
           aliceVision_fuseCut
           aliceVision_sfmData
           aliceVision_sfmDataIO
           Boost::program_options
           Boost::json
    )

    # Lidar decimation
    alicevision_add_software(aliceVision_lidarDecimating
    SOURCE main_lidarDecimating.cpp
    FOLDER ${FOLDER_SOFTWARE_PIPELINE}
    LINKS aliceVision_system
          aliceVision_cmdline
          aliceVision_mvsData
          aliceVision_mvsUtils
          aliceVision_mesh
          aliceVision_fuseCut
          aliceVision_sfmData
          aliceVision_sfmDataIO
          Boost::program_options
          Boost::json
          OpenMeshCore
          OpenMeshTools
   )

    if(ALICEVISION_HAVE_MESHSDFILTER)

        # Mesh Denoising
        alicevision_add_software(aliceVision_meshDenoising
            SOURCE main_meshDenoising.cpp
            FOLDER ${FOLDER_SOFTWARE_PIPELINE}
            LINKS aliceVision_system
                  aliceVision_cmdline
                  aliceVision_mvsUtils
                  MeshSDLibrary
                  Eigen3::Eigen
                  Boost::program_options
        )

        # Mesh Decimate
        alicevision_add_software(aliceVision_meshDecimate
            SOURCE main_meshDecimate.cpp
            FOLDER ${FOLDER_SOFTWARE_PIPELINE}
            LINKS aliceVision_system
                  aliceVision_cmdline
                  aliceVision_mvsUtils
                  OpenMeshCore
                  OpenMeshTools
                  Boost::program_options
        )
    endif()

    # Mesh Filtering
    alicevision_add_software(aliceVision_meshFiltering
        SOURCE main_meshFiltering.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_mvsUtils
              aliceVision_mesh
              Boost::program_options
    )

    # Mesh Masking
    alicevision_add_software(aliceVision_meshMasking
        SOURCE main_meshMasking.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_sfmMvsUtils
              aliceVision_mvsUtils
              aliceVision_mesh
              aliceVision_sfmData
              aliceVision_sfmDataIO
              Boost::program_options
    )

    # Mesh Resampling
    alicevision_add_software(aliceVision_meshResampling
        SOURCE main_meshResampling.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_mvsUtils
              Geogram::geogram
              Boost::program_options
    )

    # Texturing
    alicevision_add_software(aliceVision_texturing
        SOURCE main_texturing.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_mvsData
              aliceVision_sfmMvsUtils
              aliceVision_mvsUtils
              aliceVision_mesh
              aliceVision_sfmData
              aliceVision_sfmDataIO
              Boost::program_options
    )
endif() # if(ALICEVISION_BUILD_MVS)


if (ALICEVISION_BUILD_STEREOPHOTOMETRY)
    alicevision_add_software(aliceVision_lightingCalibration
        SOURCE main_lightingCalibration.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_lightingEstimation
              aliceVision_cmdline
              aliceVision_system
              aliceVision_sfmData
              aliceVision_sfmDataIO
              Boost::program_options
    )

    if(ALICEVISION_HAVE_OPENCV)
        # Photometric Stereo

        if(ALICEVISION_BUILD_MVS)
            alicevision_add_software(aliceVision_photometricStereo
                SOURCE main_photometricStereo.cpp
                FOLDER ${FOLDER_SOFTWARE_PIPELINE}
                LINKS aliceVision_photometricStereo
                      aliceVision_cmdline
                      aliceVision_system
                      aliceVision_mvsData
                      aliceVision_mvsUtils
                      aliceVision_sfmData
                      aliceVision_sfmDataIO
                      Boost::program_options
            )

            alicevision_add_software(aliceVision_normalIntegration
                SOURCE main_normalIntegration.cpp
                FOLDER ${FOLDER_SOFTWARE_PIPELINE}
                LINKS aliceVision_photometricStereo
                      aliceVision_cmdline
                      aliceVision_system
                      aliceVision_mvsData
                      aliceVision_mvsUtils
                      aliceVision_sfmData
                      aliceVision_sfmDataIO
                      Boost::program_options
            )
        endif()
        
        if(ALICEVISION_HAVE_ONNX)
            # SphereDetection
            alicevision_add_software(aliceVision_sphereDetection
                SOURCE main_sphereDetection.cpp
                FOLDER ${FOLDER_SOFTWARE_PIPELINE}
                LINKS aliceVision_image
                      aliceVision_cmdline
                      aliceVision_system
                      aliceVision_sphereDetection
                      aliceVision_sfmData
                      aliceVision_sfmDataIO
                      ${OpenCV_LIBRARIES}
            )
        endif()
    endif()
endif()

if(ALICEVISION_BUILD_HDR)
    alicevision_add_software(aliceVision_LdrToHdrSampling
        SOURCE main_LdrToHdrSampling.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_image
              aliceVision_hdr
              aliceVision_sfmData
              aliceVision_sfmDataIO
              Boost::program_options
    )

    alicevision_add_software(aliceVision_LdrToHdrCalibration
        SOURCE main_LdrToHdrCalibration.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_image
              aliceVision_hdr
              aliceVision_sfmData
              aliceVision_sfmDataIO
              Boost::program_options
    )

    alicevision_add_software(aliceVision_LdrToHdrMerge
        SOURCE main_LdrToHdrMerge.cpp
        FOLDER ${FOLDER_SOFTWARE_PIPELINE}
        LINKS aliceVision_system
              aliceVision_cmdline
              aliceVision_image
              aliceVision_hdr
              aliceVision_sfmData
              aliceVision_sfmDataIO
              Boost::program_options
    )

endif()


if(ALICEVISION_BUILD_SEGMENTATION)
    if(ALICEVISION_HAVE_ONNX)
        # Image Segmentation
        alicevision_add_software(aliceVision_imageSegmentation
            SOURCE main_imageSegmentation.cpp
            FOLDER ${FOLDER_SOFTWARE_PIPELINE}
            LINKS aliceVision_image
                  aliceVision_cmdline
                  aliceVision_system
                  aliceVision_sfmData
                  aliceVision_sfmDataIO
                  aliceVision_segmentation
        )
    endif()
endif()
