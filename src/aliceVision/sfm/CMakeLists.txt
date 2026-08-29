# Headers
set(sfm_bundle_files_headers
    bundle/BundleAdjustment.hpp
    bundle/BundleAdjustmentCeres.hpp
    utils/statistics.hpp
    utils/poseFilter.hpp
)

# Sources
set(sfm_bundle_files_sources
    bundle/BundleAdjustmentCeres.cpp
    utils/statistics.cpp
    utils/poseFilter.cpp
)

set(sfm_files_headers
    pipeline/global/GlobalSfMRotationAveragingSolver.hpp
    pipeline/global/GlobalSfMTranslationAveragingSolver.hpp
    pipeline/global/MutexSet.hpp
    pipeline/global/ReconstructionEngine_globalSfM.hpp
    pipeline/global/reindexGlobalSfM.hpp
    pipeline/global/TranslationTripletKernelACRansac.hpp
    pipeline/localization/SfMLocalizer.hpp
    pipeline/sequential/ReconstructionEngine_sequentialSfM.hpp
    pipeline/ReconstructionEngine.hpp
    pipeline/RigSequence.hpp
    pipeline/pairwiseMatchesIO.hpp
    pipeline/RelativePoseInfo.hpp
    pipeline/structureFromKnownPoses/StructureEstimationFromKnownPoses.hpp
    pipeline/panorama/ReconstructionEngine_panorama.hpp
    pipeline/bootstrapping/EstimateAngle.hpp
    pipeline/bootstrapping/PairsScoring.hpp
    pipeline/bootstrapping/Bootstrap.hpp
    pipeline/bootstrapping/TracksDepths.hpp
    pipeline/expanding/SfmTriangulation.hpp
    pipeline/expanding/SfmResection.hpp
    pipeline/expanding/SfmBundle.hpp
    pipeline/expanding/ExpansionHistory.hpp
    pipeline/expanding/ExpansionChunk.hpp
    pipeline/expanding/ExpansionIteration.hpp
    pipeline/expanding/ExpansionPostProcess.hpp
    pipeline/expanding/ExpansionPostProcessRig.hpp
    pipeline/expanding/ExpansionPolicy.hpp
    pipeline/expanding/ExpansionPolicyLegacy.hpp
    pipeline/expanding/ExpansionPolicySequence.hpp
    pipeline/expanding/ExpansionProcess.hpp
    pipeline/expanding/ConnexityGraph.hpp
    pipeline/expanding/LbaPolicy.hpp
    pipeline/expanding/LbaPolicyConnexity.hpp
    pipeline/positioning/GlobalPositioning.hpp
    pipeline/regionsIO.hpp
    utils/alignment.hpp
    utils/poseNoise.hpp
    utils/preprocess.hpp
    utils/syntheticScene.hpp
    utils/gauge.hpp
    LocalBundleAdjustmentGraph.hpp
    FrustumFilter.hpp
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
    pipeline/sequential/ReconstructionEngine_sequentialSfM.cpp
    pipeline/ReconstructionEngine.cpp
    pipeline/RigSequence.cpp
    pipeline/RelativePoseInfo.cpp
    pipeline/structureFromKnownPoses/StructureEstimationFromKnownPoses.cpp
    pipeline/panorama/ReconstructionEngine_panorama.cpp
    pipeline/bootstrapping/EstimateAngle.cpp
    pipeline/bootstrapping/PairsScoring.cpp
    pipeline/bootstrapping/Bootstrap.cpp
    pipeline/bootstrapping/TracksDepths.cpp
    pipeline/expanding/SfmTriangulation.cpp
    pipeline/expanding/SfmResection.cpp
    pipeline/expanding/SfmBundle.cpp
    pipeline/expanding/DistanceWeighting.cpp
    pipeline/expanding/ExpansionHistory.cpp
    pipeline/expanding/ExpansionChunk.cpp
    pipeline/expanding/ExpansionIteration.cpp
    pipeline/expanding/ExpansionPolicyLegacy.cpp
    pipeline/expanding/ExpansionPolicySequence.cpp
    pipeline/expanding/ExpansionPostProcessRig.cpp
    pipeline/expanding/ExpansionProcess.cpp
    pipeline/expanding/ConnexityGraph.cpp
    pipeline/expanding/LbaPolicyConnexity.cpp
    pipeline/expanding/LocalizationValidationPolicyLegacy.cpp
    pipeline/positioning/GlobalPositioning.cpp
    pipeline/regionsIO.cpp
    utils/alignment.cpp
    utils/poseNoise.cpp
    utils/preprocess.cpp
    utils/syntheticScene.cpp
    utils/gauge.cpp
    LocalBundleAdjustmentGraph.cpp
    FrustumFilter.cpp
    generateReport.cpp
    sfmFilters.cpp
    sfmStatistics.cpp
    sfmTriangulation.cpp
)

alicevision_add_library(aliceVision_sfm_bundle
    SOURCES ${sfm_bundle_files_headers} ${sfm_bundle_files_sources}
    PUBLIC_LINKS
        aliceVision_system
        aliceVision_numeric
        aliceVision_geometry
        aliceVision_camera
        aliceVision_sfmData
        Boost::boost
        ${CERES_LIBRARIES}
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
        aliceVision_sfm_bundle
        aliceVision_dataio
        Boost::boost
    PRIVATE_LINKS
        ${LEMON_LIBRARY}
        nanoflann::nanoflann
)

# Unit tests

alicevision_add_test(bundle/bundleAdjustment_test.cpp
    NAME "sfm_bundleAdjustment"
    LINKS aliceVision_sfm
          aliceVision_multiview
          aliceVision_multiview_test_data
          aliceVision_feature
          aliceVision_system
          ${LEMON_LIBRARY}
)

alicevision_add_test(bundle/bundleAdjustment_Enhanced_test.cpp
    NAME "sfm_bundleAdjustment_Enhanced"
    LINKS aliceVision_sfm
          aliceVision_multiview
          aliceVision_multiview_test_data
          aliceVision_feature
          aliceVision_system
          ${LEMON_LIBRARY}
)

alicevision_add_test(bundle/bundleAdjustment_temporalConstraint_test.cpp
    NAME "sfm_bundleAdjustment_temporalConstraint"
    LINKS aliceVision_sfm
          aliceVision_track
)

alicevision_add_test(utils/alignment_test.cpp
    NAME "sfm_alignment"
    LINKS
        aliceVision_sfm
        aliceVision_multiview
        aliceVision_multiview_test_data
        ${LEMON_LIBRARY}
)

alicevision_add_test(utils/poseFilter_test.cpp
    NAME "sfm_poseFilter"
    LINKS
        aliceVision_sfm
)

add_subdirectory(pipeline)

# SWIG Binding
if (ALICEVISION_BUILD_SWIG_BINDING)
    alicevision_swig_add_library(sfm
        SOURCES sfm.i
        PUBLIC_LINKS
            aliceVision_sfmData
            aliceVision_sfm
            aliceVision_sfm_bundle
            ${Python3_LIBRARIES}
        PRIVATE_INCLUDE_DIRS
            ../include
            ${ALICEVISION_ROOT}/include
            ${Python3_INCLUDE_DIRS}
            ${Python3_NumPy_INCLUDE_DIRS}
    )
endif()
