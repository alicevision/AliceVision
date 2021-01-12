# Headers
set(sfm_files_headers
  pipeline/global/GlobalSfMRotationAveragingSolver.hpp
  pipeline/global/GlobalSfMTranslationAveragingSolver.hpp
  pipeline/global/MutexSet.hpp
  pipeline/global/ReconstructionEngine_globalSfM.hpp
  pipeline/global/reindexGlobalSfM.hpp
  pipeline/global/TranslationTripletKernelACRansac.hpp
  pipeline/localization/SfMLocalizer.hpp
  pipeline/localization/SfMLocalizationSingle3DTrackObservationDatabase.hpp
  pipeline/sequential/ReconstructionEngine_sequentialSfM.hpp
  pipeline/ReconstructionEngine.hpp
  pipeline/RigSequence.hpp
  pipeline/pairwiseMatchesIO.hpp
  pipeline/RelativePoseInfo.hpp
  pipeline/structureFromKnownPoses/StructureEstimationFromKnownPoses.hpp
  pipeline/panorama/ReconstructionEngine_panorama.hpp
  pipeline/regionsIO.hpp
  utils/alignment.hpp
  utils/statistics.hpp
  utils/syntheticScene.hpp
  BundleAdjustment.hpp
  BundleAdjustmentCeres.hpp
  BundleAdjustmentPanoramaCeres.hpp
  BundleAdjustmentSymbolicCeres.hpp
  LocalBundleAdjustmentGraph.hpp
  FrustumFilter.hpp
  ResidualErrorFunctor.hpp
  filters.hpp
  generateReport.hpp
  sfm.hpp
  sfmFilters.hpp
  sfmStatistics.hpp
  sfmTriangulation.hpp
)

# Sources
set(sfm_files_sources
  pipeline/global/GlobalSfMRotationAveragingSolver.cpp
  pipeline/global/GlobalSfMTranslationAveragingSolver.cpp
  pipeline/global/ReconstructionEngine_globalSfM.cpp
  pipeline/localization/SfMLocalizer.cpp
  pipeline/localization/SfMLocalizationSingle3DTrackObservationDatabase.cpp
  pipeline/sequential/ReconstructionEngine_sequentialSfM.cpp
  pipeline/ReconstructionEngine.cpp
  pipeline/RigSequence.cpp
  pipeline/RelativePoseInfo.cpp
  pipeline/structureFromKnownPoses/StructureEstimationFromKnownPoses.cpp
  pipeline/panorama/ReconstructionEngine_panorama.cpp
  pipeline/regionsIO.cpp
  utils/alignment.cpp
  utils/statistics.cpp
  utils/syntheticScene.cpp
  BundleAdjustmentCeres.cpp
  BundleAdjustmentPanoramaCeres.cpp
  BundleAdjustmentSymbolicCeres.cpp
  LocalBundleAdjustmentGraph.cpp
  FrustumFilter.cpp
  generateReport.cpp
  sfmFilters.cpp
  sfmStatistics.cpp
  sfmTriangulation.cpp
)

alicevision_add_library(aliceVision_sfm
  SOURCES ${sfm_files_headers} ${sfm_files_sources}
  PUBLIC_LINKS
    aliceVision_system
    aliceVision_numeric
    aliceVision_multiview
    aliceVision_geometry
    aliceVision_camera
    aliceVision_matching
    aliceVision_feature
    aliceVision_graph
    aliceVision_track
    aliceVision_lInftyComputerVision
    aliceVision_linearProgramming
    aliceVision_sfmData
    aliceVision_sfmDataIO
    Boost::filesystem
    Boost::boost
    ${CERES_LIBRARIES}
)

# Unit tests

alicevision_add_test(bundleAdjustment_test.cpp
  NAME "sfm_bundleAdjustment"
  LINKS aliceVision_sfm
        aliceVision_multiview
        aliceVision_multiview_test_data
        aliceVision_feature
        aliceVision_system
)

add_subdirectory(pipeline)

