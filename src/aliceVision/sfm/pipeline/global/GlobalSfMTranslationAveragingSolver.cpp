// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/pipeline/global/GlobalSfMTranslationAveragingSolver.hpp>
#include <aliceVision/sfm/filters.hpp>
#include <aliceVision/sfm/sfmTriangulation.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustmentCeres.hpp>
#include <aliceVision/sfm/pipeline/global/reindexGlobalSfM.hpp>
#include <aliceVision/sfm/pipeline/global/MutexSet.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/multiview/translationAveraging/common.hpp>
#include <aliceVision/multiview/translationAveraging/solver.hpp>
#include <aliceVision/graph/graph.hpp>
#include <aliceVision/track/TracksBuilder.hpp>
#include <aliceVision/stl/stl.hpp>
#include <aliceVision/system/ProgressDisplay.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/linearProgramming/linearProgramming.hpp>
#include <aliceVision/multiview/essential.hpp>
#include <aliceVision/robustEstimation/conditioning.hpp>
#include <aliceVision/multiview/translationAveraging/common.hpp>
#include <aliceVision/multiview/translationAveraging/solver.hpp>
#include <aliceVision/sfm/pipeline/global/TranslationTripletKernelACRansac.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/alicevision_omp.hpp>

#include <aliceVision/utils/Histogram.hpp>

namespace aliceVision {
namespace sfm {

using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfmData;

/// Use features in normalized camera frames
bool GlobalSfMTranslationAveragingSolver::run(ETranslationAveragingMethod eTranslationAveragingMethod,
                                              SfMData& sfmData,
                                              const feature::FeaturesPerView& normalizedFeaturesPerView,
                                              const matching::PairwiseMatches& pairwiseMatches,
                                              const std::map<IndexT, Mat3>& mapGlobalR,
                                              std::mt19937& randomNumberGenerator,
                                              matching::PairwiseMatches& tripletWiseMatches)
{
    // Compute the relative translations and save them to vec_initialRijTijEstimates:
    computeTranslations(sfmData, normalizedFeaturesPerView, pairwiseMatches, mapGlobalR, randomNumberGenerator, tripletWiseMatches);

    const bool translation = translationAveraging(eTranslationAveragingMethod, sfmData, mapGlobalR);

    // Filter matches to keep only them link to view that have valid poses
    // (necessary since multiple components exists before translation averaging)
    std::set<IndexT> validViewIds;
    for (const auto& view : sfmData.getViews())
    {
        if (sfmData.isPoseAndIntrinsicDefined(view.second.get()))
            validViewIds.insert(view.first);
    }
    keepOnlyReferencedElement(validViewIds, tripletWiseMatches);

    return translation;
}

bool GlobalSfMTranslationAveragingSolver::translationAveraging(ETranslationAveragingMethod eTranslationAveragingMethod,
                                                               SfMData& sfmData,
                                                               const std::map<IndexT, Mat3>& mapGlobalR)
{
    //-------------------
    //-- GLOBAL TRANSLATIONS ESTIMATION from initial triplets t_ij guess
    //-------------------

    // Keep the largest Biedge connected component graph of relative translations
    PairSet pairs;
    std::transform(m_vec_initialRijTijEstimates.begin(), m_vec_initialRijTijEstimates.end(), std::inserter(pairs, pairs.begin()), stl::RetrieveKey());
    const std::set<IndexT> setRemainingIds = aliceVision::graph::CleanGraph_KeepLargestBiEdge_Nodes<PairSet, IndexT>(pairs, "./");
    keepOnlyReferencedElement(setRemainingIds, m_vec_initialRijTijEstimates);

    {
        const std::set<IndexT> index = translationAveraging::getIndexT(m_vec_initialRijTijEstimates);

        const size_t iNview = index.size();
        ALICEVISION_LOG_DEBUG("\n-------------------------------\n"
                              " Global translations computation:\n"
                              "   - Ready to compute "
                              << iNview << " global translations."
                              << "\n"
                                 "     from #relative translations: "
                              << m_vec_initialRijTijEstimates.size());

        if (iNview < 3)
        {
            // Too tiny image set to perform motion averaging
            return false;
        }
        //-- Update initial estimates from [minId,maxId] to range [0->Ncam]
        translationAveraging::RelativeInfoVec vecInitialRijTijEstimatesCpy = m_vec_initialRijTijEstimates;
        const PairSet pairs = translationAveraging::getPairs(vecInitialRijTijEstimatesCpy);
        std::map<IndexT, IndexT> _reindexForward, _reindexBackward;
        reindex(pairs, _reindexForward, _reindexBackward);
        for (size_t i = 0; i < vecInitialRijTijEstimatesCpy.size(); ++i)
        {
            aliceVision::translationAveraging::relativeInfo& rel = vecInitialRijTijEstimatesCpy[i];
            rel.first = Pair(_reindexForward[rel.first.first], _reindexForward[rel.first.second]);
        }

        aliceVision::system::Timer timerLPTranslation;

        switch (eTranslationAveragingMethod)
        {
            case TRANSLATION_AVERAGING_L1:
            {
                double gamma = -1.0;
                std::vector<double> vecSolution;
                {
                    vecSolution.resize(iNview * 3 + vecInitialRijTijEstimatesCpy.size() / 3 + 1);
                    using namespace aliceVision::linearProgramming;
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
                    MOSEKSolver solverLP(vecSolution.size());
#else
                    OSI_CISolverWrapper solverLP(vecSolution.size());
#endif

                    lInfinityCV::Tifromtij_ConstraintBuilder_OneLambdaPerTrif cstBuilder(vecInitialRijTijEstimatesCpy);

                    LPConstraintsSparse constraint;
                    //-- Setup constraint and solver
                    cstBuilder.Build(constraint);
                    solverLP.setup(constraint);
                    //--
                    // Solving
                    const bool bFeasible = solverLP.solve();
                    ALICEVISION_LOG_DEBUG(" Feasibility " << bFeasible);
                    //--
                    if (bFeasible)
                    {
                        solverLP.getSolution(vecSolution);
                        gamma = vecSolution[vecSolution.size() - 1];
                    }
                    else
                    {
                        ALICEVISION_LOG_WARNING("Compute global translations: failed");
                        return false;
                    }
                }

                const double timeLPTranslation = timerLPTranslation.elapsed();
                //-- Export triplet statistics:
                {
                    std::ostringstream os;
                    os << "Translation fusion statistics.";
                    os.str("");
                    os << "-------------------------------\n"
                       << "-- #relative estimates: " << vecInitialRijTijEstimatesCpy.size() << " converge with gamma: " << gamma << ".\n"
                       << " timing (s): " << timeLPTranslation << ".\n"
                       << "-------------------------------"
                       << "\n";
                    ALICEVISION_LOG_DEBUG(os.str());
                }

                ALICEVISION_LOG_DEBUG("Found solution:\n" << vecSolution);

                std::vector<double> vecCamTranslation(iNview * 3, 0);
                std::copy(&vecSolution[0], &vecSolution[iNview * 3], &vecCamTranslation[0]);

                std::vector<double> vecCamRelLambdas(&vecSolution[iNview * 3], &vecSolution[iNview * 3 + vecInitialRijTijEstimatesCpy.size() / 3]);

                ALICEVISION_LOG_DEBUG("cam position: " << vecCamTranslation);
                ALICEVISION_LOG_DEBUG("cam Lambdas: " << vecCamRelLambdas);

                // Update the view poses according the found camera centers
                for (std::size_t i = 0; i < iNview; ++i)
                {
                    const Vec3 t(vecCamTranslation[i * 3], vecCamTranslation[i * 3 + 1], vecCamTranslation[i * 3 + 2]);
                    const IndexT poseId = _reindexBackward[i];
                    const Mat3& Ri = mapGlobalR.at(poseId);
                    sfmData.setAbsolutePose(poseId, CameraPose(Pose3(Ri, -Ri.transpose() * t)));
                }
            }
            break;

            case TRANSLATION_AVERAGING_SOFTL1:
            {
                std::vector<Vec3> vecTranslations;
                if (!translationAveraging::solve_translations_problem_softl1(vecInitialRijTijEstimatesCpy, true, iNview, vecTranslations))
                {
                    ALICEVISION_LOG_WARNING("Compute global translations: failed");
                    return false;
                }

                // A valid solution was found:
                // - Update the view poses according the found camera translations
                for (std::size_t i = 0; i < iNview; ++i)
                {
                    const Vec3& t = vecTranslations[i];
                    const IndexT poseId = _reindexBackward[i];
                    const Mat3& Ri = mapGlobalR.at(poseId);
                    sfmData.setAbsolutePose(poseId, CameraPose(Pose3(Ri, -Ri.transpose() * t)));
                }
            }
            break;

            case TRANSLATION_AVERAGING_L2_DISTANCE_CHORDAL:
            {
                std::vector<int> vecEdges;
                vecEdges.reserve(vecInitialRijTijEstimatesCpy.size() * 2);
                std::vector<double> vecPoses;
                vecPoses.reserve(vecInitialRijTijEstimatesCpy.size() * 3);
                std::vector<double> vecWeights;
                vecWeights.reserve(vecInitialRijTijEstimatesCpy.size());

                for (int i = 0; i < vecInitialRijTijEstimatesCpy.size(); ++i)
                {
                    const aliceVision::translationAveraging::relativeInfo& rel = vecInitialRijTijEstimatesCpy[i];
                    vecEdges.push_back(rel.first.first);
                    vecEdges.push_back(rel.first.second);
                    // Since index have been remapped
                    // (use the backward indexing to retrieve the second global rotation)
                    const IndexT secondId = _reindexBackward[rel.first.second];
                    const View* view = sfmData.getViews().at(secondId).get();
                    const Mat3& Ri = mapGlobalR.at(view->getPoseId());
                    const Vec3 direction = -(Ri.transpose() * rel.second.second.normalized());

                    vecPoses.push_back(direction(0));
                    vecPoses.push_back(direction(1));
                    vecPoses.push_back(direction(2));

                    vecWeights.push_back(1.0);
                }

                const double functionTolerance = 1e-7, parameterTolerance = 1e-8;
                const int maxIterations = 500;

                const double lossWidth = 0.0;  // No loss in order to compare with TRANSLATION_AVERAGING_L1

                std::vector<double> X(iNview * 3, 0.0);
                if (!translationAveraging::solve_translations_problem_l2_chordal(&vecEdges[0],
                                                                                 &vecPoses[0],
                                                                                 &vecWeights[0],
                                                                                 vecInitialRijTijEstimatesCpy.size(),
                                                                                 lossWidth,
                                                                                 &X[0],
                                                                                 functionTolerance,
                                                                                 parameterTolerance,
                                                                                 maxIterations))
                {
                    ALICEVISION_LOG_WARNING("Compute global translations: failed");
                    return false;
                }

                // Update camera center for each view
                for (std::size_t i = 0; i < iNview; ++i)
                {
                    const Vec3 C(X[i * 3], X[i * 3 + 1], X[i * 3 + 2]);
                    const IndexT poseId = _reindexBackward[i];  // undo the reindexing
                    const Mat3& Ri = mapGlobalR.at(poseId);
                    sfmData.setAbsolutePose(poseId, CameraPose(Pose3(Ri, C)));
                }
            }
            break;
            default:
            {
                ALICEVISION_LOG_WARNING("Unknown translation averaging method");
                return false;
            }
        }
    }
    return true;
}

void GlobalSfMTranslationAveragingSolver::computeTranslations(const SfMData& sfmData,
                                                              const feature::FeaturesPerView& normalizedFeaturesPerView,
                                                              const matching::PairwiseMatches& pairwiseMatches,
                                                              const std::map<IndexT, Mat3>& mapGlobalR,
                                                              std::mt19937& randomNumberGenerator,
                                                              matching::PairwiseMatches& tripletWiseMatches)
{
    ALICEVISION_LOG_DEBUG("-------------------------------\n"
                          " Relative translations computation:\n"
                          "-------------------------------");

    // Compute relative translations over the graph of global rotations
    //  thanks to an edge coverage algorithm
    computePutativeTranslationEdgesCoverage(
      sfmData, mapGlobalR, normalizedFeaturesPerView, pairwiseMatches, randomNumberGenerator, m_vec_initialRijTijEstimates, tripletWiseMatches);
}

//-- Perform a trifocal estimation of the graph contained in vec_triplets with an
// edge coverage algorithm. Its complexity is sub-linear in term of edges count.
void GlobalSfMTranslationAveragingSolver::computePutativeTranslationEdgesCoverage(const SfMData& sfmData,
                                                                                  const std::map<IndexT, Mat3>& mapGlobalR,
                                                                                  const feature::FeaturesPerView& normalizedFeaturesPerView,
                                                                                  const matching::PairwiseMatches& pairwiseMatches,
                                                                                  std::mt19937& randomNumberGenerator,
                                                                                  translationAveraging::RelativeInfoVec& vecInitialEstimates,
                                                                                  matching::PairwiseMatches& newpairMatches)
{
    aliceVision::system::Timer timerLPTriplet;

    //--
    // Compute the relative translations using triplets of rotations over the rotation graph.
    //--
    //
    // 1. List plausible triplets over the global rotation pose graph Ids.
    //   - list all edges that have support in the rotation pose graph
    //
    PairSet rotationPoseIdGraph;
    std::set<IndexT> setPoseIds;
    std::transform(mapGlobalR.begin(), mapGlobalR.end(), std::inserter(setPoseIds, setPoseIds.begin()), stl::RetrieveKey());
    // List shared correspondences (pairs) between poses
    for (const auto& matchIterator : pairwiseMatches)
    {
        const Pair pair = matchIterator.first;
        const View* v1 = sfmData.getViews().at(pair.first).get();
        const View* v2 = sfmData.getViews().at(pair.second).get();

        if (  // Consider the pair iff it is supported by the rotation graph
          (v1->getPoseId() != v2->getPoseId()) && setPoseIds.count(v1->getPoseId()) && setPoseIds.count(v2->getPoseId()))
        {
            rotationPoseIdGraph.insert(std::make_pair(v1->getPoseId(), v2->getPoseId()));
        }
    }
    // List putative triplets (from global rotations Ids)
    const std::vector<graph::Triplet> vecTriplets = graph::tripletListing(rotationPoseIdGraph);
    ALICEVISION_LOG_DEBUG("#Triplets: " << vecTriplets.size());

    {
        // Compute triplets of translations
        // Avoid to cover each edge of the graph by using an edge coverage algorithm
        // An estimated triplets of translation mark three edges as estimated.

        //-- precompute the number of track per triplet:
        std::map<IndexT, IndexT> mapTracksPerTriplets;

#pragma omp parallel for schedule(dynamic)
        for (int i = 0; i < (int)vecTriplets.size(); ++i)
        {
            // List matches that belong to the triplet of poses
            const graph::Triplet& triplet = vecTriplets[i];
            matching::PairwiseMatches mapTripletMatches;
            const std::set<IndexT> setTripletPoseIds = {triplet.i, triplet.j, triplet.k};
            // List shared correspondences (pairs) between the triplet poses
            for (const auto& matchIterator : pairwiseMatches)
            {
                const Pair pair = matchIterator.first;
                const View* v1 = sfmData.getViews().at(pair.first).get();
                const View* v2 = sfmData.getViews().at(pair.second).get();
                if (  // Consider the pair iff it is supported by the triplet graph & 2 different pose id
                  (v1->getPoseId() != v2->getPoseId()) && setTripletPoseIds.count(v1->getPoseId()) && setTripletPoseIds.count(v2->getPoseId()))
                {
                    mapTripletMatches.insert(matchIterator);
                }
            }
            // Compute tracks:
            {
                aliceVision::track::TracksBuilder tracksBuilder;
                tracksBuilder.build(mapTripletMatches);
                tracksBuilder.filter(true, 3);

#pragma omp critical
                mapTracksPerTriplets[i] = tracksBuilder.nbTracks();  // count the # of matches in the UF tree
            }
        }

        typedef Pair myEdge;

        //-- Alias (list triplet ids used per pose id edges)
        std::map<myEdge, std::vector<size_t>> mapTripletIdsPerEdge;
        for (size_t i = 0; i < vecTriplets.size(); ++i)
        {
            const graph::Triplet& triplet = vecTriplets[i];
            mapTripletIdsPerEdge[std::make_pair(triplet.i, triplet.j)].push_back(i);
            mapTripletIdsPerEdge[std::make_pair(triplet.i, triplet.k)].push_back(i);
            mapTripletIdsPerEdge[std::make_pair(triplet.j, triplet.k)].push_back(i);
        }

        // Collect edges that are covered by the triplets
        std::vector<myEdge> vecEdges;
        std::transform(mapTripletIdsPerEdge.begin(), mapTripletIdsPerEdge.end(), std::back_inserter(vecEdges), stl::RetrieveKey());

        aliceVision::sfm::MutexSet<myEdge> mMutexSet;

        auto progressDisplay =
          system::createConsoleProgressDisplay(vecEdges.size(), std::cout, "\nRelative translations computation (edge coverage algorithm)\n");

        // set number of threads, 1 if openMP is not enabled
        std::vector<translationAveraging::RelativeInfoVec> initialEstimates(omp_get_max_threads());
        const bool bVerbose = false;

#pragma omp parallel for schedule(dynamic)
        for (int k = 0; k < vecEdges.size(); ++k)
        {
            const myEdge& edge = vecEdges[k];
            ++progressDisplay;
            if (mMutexSet.count(edge) == 0 && mMutexSet.size() != vecEdges.size())
            {
                // Find the triplets that support the given edge
                const auto& vecPossibleTripletIndexes = mapTripletIdsPerEdge.at(edge);

                //-- Sort the triplets according the number of track they are supporting
                std::vector<size_t> vecCommonTracksPerTriplets;
                for (const size_t tripletIndex : vecPossibleTripletIndexes)
                {
                    vecCommonTracksPerTriplets.push_back(mapTracksPerTriplets[tripletIndex]);
                }

                using namespace stl::indexed_sort;
                std::vector<sort_index_packet_descend<size_t, size_t>> packetVec(vecCommonTracksPerTriplets.size());
                sort_index_helper(packetVec, &vecCommonTracksPerTriplets[0]);

                std::vector<size_t> vecTripletOrdered(vecCommonTracksPerTriplets.size(), 0);
                for (size_t i = 0; i < vecCommonTracksPerTriplets.size(); ++i)
                {
                    vecTripletOrdered[i] = vecPossibleTripletIndexes[packetVec[i].index];
                }

                // Try to solve a triplet of translations for the given edge
                for (const size_t tripletIndex : vecTripletOrdered)
                {
                    const graph::Triplet& triplet = vecTriplets[tripletIndex];

                    // If the triplet is already estimated by another thread; try the next one
                    if (mMutexSet.count(Pair(triplet.i, triplet.j)) && mMutexSet.count(Pair(triplet.i, triplet.k)) &&
                        mMutexSet.count(Pair(triplet.j, triplet.k)))
                    {
                        break;
                    }

                    //--
                    // Try to estimate this triplet of translations
                    //--
                    double dPrecision = 4.0;  // upper bound of the residual pixel reprojection error

                    std::vector<Vec3> vecTis(3);
                    std::vector<size_t> vecInliers;
                    aliceVision::track::TracksMap poseTripletTracks;

                    const std::string sOutDirectory = "./";
                    const bool bTripletEstimation = estimateTTriplet(sfmData,
                                                                     mapGlobalR,
                                                                     normalizedFeaturesPerView,
                                                                     pairwiseMatches,
                                                                     triplet,
                                                                     randomNumberGenerator,
                                                                     vecTis,
                                                                     dPrecision,
                                                                     vecInliers,
                                                                     poseTripletTracks,
                                                                     sOutDirectory);

                    if (bTripletEstimation)
                    {
                        // Since new translation edges have been computed, mark their corresponding edges as estimated
                        mMutexSet.insert(std::make_pair(triplet.i, triplet.j));
                        mMutexSet.insert(std::make_pair(triplet.j, triplet.k));
                        mMutexSet.insert(std::make_pair(triplet.i, triplet.k));

                        // Compute the triplet relative motions (IJ, JK, IK)
                        {
                            const Mat3 RI = mapGlobalR.at(triplet.i), RJ = mapGlobalR.at(triplet.j), RK = mapGlobalR.at(triplet.k);
                            const Vec3 ti = vecTis[0], tj = vecTis[1], tk = vecTis[2];

                            Mat3 Rij;
                            Vec3 tij;
                            relativeCameraMotion(RI, ti, RJ, tj, &Rij, &tij);

                            Mat3 Rjk;
                            Vec3 tjk;
                            relativeCameraMotion(RJ, tj, RK, tk, &Rjk, &tjk);

                            Mat3 Rik;
                            Vec3 tik;
                            relativeCameraMotion(RI, ti, RK, tk, &Rik, &tik);

                            // set number of threads, 1 if openMP is not enabled
                            const int threadId = omp_get_thread_num();

                            initialEstimates[threadId].emplace_back(std::make_pair(triplet.i, triplet.j), std::make_pair(Rij, tij));
                            initialEstimates[threadId].emplace_back(std::make_pair(triplet.j, triplet.k), std::make_pair(Rjk, tjk));
                            initialEstimates[threadId].emplace_back(std::make_pair(triplet.i, triplet.k), std::make_pair(Rik, tik));

//--- ATOMIC
#pragma omp critical
                            {
                                // Add inliers as valid pairwise matches
                                for (std::vector<size_t>::const_iterator iterInliers = vecInliers.begin(); iterInliers != vecInliers.end();
                                     ++iterInliers)
                                {
                                    using namespace aliceVision::track;
                                    TracksMap::iterator itTracks = poseTripletTracks.begin();
                                    std::advance(itTracks, *iterInliers);
                                    const Track& track = itTracks->second;

                                    // create pairwise matches from inlier track
                                    for (size_t indexI = 0; indexI < track.featPerView.size(); ++indexI)
                                    {
                                        Track::TrackInfoPerView::const_iterator iterI = track.featPerView.begin();
                                        std::advance(iterI, indexI);

                                        // extract camera indexes
                                        const size_t idViewI = iterI->first;
                                        const size_t idFeatI = iterI->second.featureId;

                                        // loop on subtracks
                                        for (size_t indexJ = indexI + 1; indexJ < track.featPerView.size(); ++indexJ)
                                        {
                                            Track::TrackInfoPerView::const_iterator iterJ = track.featPerView.begin();
                                            std::advance(iterJ, indexJ);

                                            // extract camera indexes
                                            const size_t idViewJ = iterJ->first;
                                            const size_t idFeatJ = iterJ->second.featureId;

                                            newpairMatches[std::make_pair(idViewI, idViewJ)][track.descType].emplace_back(idFeatI, idFeatJ);
                                        }
                                    }
                                }
                            }
                        }
                        // Since a relative translation have been found for the edge: vec_edges[k],
                        //  we break and start to estimate the translations for some other edges.
                        break;
                    }
                }
            }
        }
        // Merge thread estimates
        for (const auto vec : initialEstimates)
        {
            for (const auto val : vec)
            {
                vecInitialEstimates.emplace_back(val);
            }
        }
    }

    const double timeLPTriplet = timerLPTriplet.elapsed();
    ALICEVISION_LOG_DEBUG("TRIPLET COVERAGE TIMING: " << timeLPTriplet << " seconds");

    ALICEVISION_LOG_DEBUG("-------------------------------\n"
                          "-- #Relative translations estimates: "
                          << m_vec_initialRijTijEstimates.size() / 3 << " computed from " << vecTriplets.size()
                          << " triplets.\n"
                             "-- resulting in "
                          << m_vec_initialRijTijEstimates.size()
                          << " translations estimation.\n"
                             "-- timing to obtain the relative translations: "
                          << timeLPTriplet
                          << " seconds.\n"
                             "-------------------------------");
}

// Robust estimation and refinement of a translation and 3D points of an image triplets.
bool GlobalSfMTranslationAveragingSolver::estimateTTriplet(const SfMData& sfmData,
                                                           const std::map<IndexT, Mat3>& mapGlobalR,
                                                           const feature::FeaturesPerView& normalizedFeaturesPerView,
                                                           const matching::PairwiseMatches& pairwiseMatches,
                                                           const graph::Triplet& posesId,
                                                           std::mt19937& randomNumberGenerator,
                                                           std::vector<Vec3>& vecTis,
                                                           double& precision,  // UpperBound of the precision found by the AContrario estimator
                                                           std::vector<std::size_t>& vecInliers,
                                                           aliceVision::track::TracksMap& tracks,
                                                           const std::string& outDirectory) const
{
    // List matches that belong to the triplet of poses
    matching::PairwiseMatches mapTripletMatches;
    const std::set<IndexT> setPoseIds = {posesId.i, posesId.j, posesId.k};
    // List shared correspondences (pairs) between poses
    for (const auto& matchIterator : pairwiseMatches)
    {
        const Pair& pair = matchIterator.first;
        const View* v1 = sfmData.getViews().at(pair.first).get();
        const View* v2 = sfmData.getViews().at(pair.second).get();
        if (  // Consider the pair iff it is supported by the triplet graph & 2 different pose id
          (v1->getPoseId() != v2->getPoseId()) && setPoseIds.count(v1->getPoseId()) && setPoseIds.count(v2->getPoseId()))
        {
            mapTripletMatches.insert(matchIterator);
        }
    }

    aliceVision::track::TracksBuilder tracksBuilder;
    tracksBuilder.build(mapTripletMatches);
    tracksBuilder.filter(true, 3);
    tracksBuilder.exportToSTL(tracks);

    if (tracks.size() < 30)
        return false;

    // Convert data
    Mat x1(2, tracks.size());
    Mat x2(2, tracks.size());
    Mat x3(2, tracks.size());

    Mat* xxx[3] = {&x1, &x2, &x3};
    std::set<IndexT> intrinsicIds;
    size_t cpt = 0;
    for (track::TracksMap::const_iterator iterTracks = tracks.begin(); iterTracks != tracks.end(); ++iterTracks, ++cpt)
    {
        const track::Track& track = iterTracks->second;
        size_t index = 0;
        for (track::Track::TrackInfoPerView::const_iterator iter = track.featPerView.begin(); iter != track.featPerView.end(); ++iter, ++index)
        {
            const size_t idxView = iter->first;
            const feature::PointFeature pt = normalizedFeaturesPerView.getFeatures(idxView, track.descType)[iter->second.featureId];
            xxx[index]->col(cpt) = pt.coords().cast<double>();
            const View* view = sfmData.getViews().at(idxView).get();
            intrinsicIds.insert(view->getIntrinsicId());
        }
    }
    // Retrieve the smallest focal value, for threshold normalization
    double minFocal = std::numeric_limits<double>::max();
    for (const auto& ids : intrinsicIds)
    {
        const camera::IntrinsicBase* intrinsicPtr = sfmData.getIntrinsics().at(ids).get();
        const camera::Pinhole* intrinsic = dynamic_cast<const camera::Pinhole*>(intrinsicPtr);
        if (intrinsic && intrinsic->isValid())
        {
            minFocal = std::min(minFocal, intrinsic->getFocalLengthPixX());
        }
    }
    if (minFocal == std::numeric_limits<double>::max())
    {
        return false;
    }

    // Get rotations:
    const std::vector<Mat3> vecGlobalRTriplet = {mapGlobalR.at(posesId.i), mapGlobalR.at(posesId.j), mapGlobalR.at(posesId.k)};

    using namespace aliceVision::trifocal;
    using namespace aliceVision::trifocal::kernel;

    typedef TranslationTripletKernelACRansac<translations_Triplet_Solver, translations_Triplet_Solver, TrifocalTensorModel> KernelType;
    const double ThresholdUpperBound = 1.0e-2;  // upper bound of the pixel residual (normalized coordinates)
    KernelType kernel(x1, x2, x3, vecGlobalRTriplet, Mat3::Identity(), ThresholdUpperBound);

    const size_t ORSA_ITER = 320;  // max number of iterations of AC-RANSAC

    TrifocalTensorModel T;
    // Note by Fabien Servant : sqrt() as it should have been Squared before ... Trying to keep the same values
    const std::pair<double, double> acStat =
      robustEstimation::ACRANSAC(kernel, randomNumberGenerator, vecInliers, ORSA_ITER, &T, sqrt(precision / minFocal));
    // If robust estimation fails => stop.
    if (precision == std::numeric_limits<double>::infinity())
        return false;

    // Update output parameters
    precision = acStat.first * minFocal;

    vecTis.resize(3);
    Mat3 K, R;
    KRt_from_P(T.P1, K, R, vecTis[0]);
    KRt_from_P(T.P2, K, R, vecTis[1]);
    KRt_from_P(T.P3, K, R, vecTis[2]);

#ifdef DEBUG_TRIPLET
    // compute 3D scene base on motion estimation
    SfMData tinyScene;

    // intialize poses (which are now shared by a group of images)
    tinyScene.poses[posesId.i] = Pose3(vecGlobalRTriplet[0], -vecGlobalRTriplet[0].transpose() * vecTis[0]);
    tinyScene.poses[posesId.j] = Pose3(vecGlobalRTriplet[1], -vecGlobalRTriplet[1].transpose() * vecTis[1]);
    tinyScene.poses[posesId.k] = Pose3(vecGlobalRTriplet[2], -vecGlobalRTriplet[2].transpose() * vecTis[2]);

    // insert views used by the relative pose pairs
    for (const auto& pairIterator : mapTripletMatches)
    {
        // initialize camera indexes
        const IndexT I = pairIterator.first.first;
        const IndexT J = pairIterator.first.second;

        // add views
        tinyScene.getViews().insert(*sfmData.getViews().find(I));
        tinyScene.getViews().insert(*sfmData.getViews().find(J));

        // add intrinsics
        const View* viewI = sfmData.getViews().at(I).get();
        const View* viewJ = sfmData.getViews().at(J).get();
        sfm_data.getIntrinsics().insert(*sfmData.getIntrinsics().find(viewI->getIntrinsicId()));
        sfm_data.getIntrinsics().insert(*sfmData.getIntrinsics().find(viewJ->getIntrinsicId()));
    }

    // Fill sfmData with the inliers tracks. Feed image observations: no 3D yet.
    Landmarks& structure = tinyScene.getLandmarks();
    for (size_t idx = 0; idx < vecInliers.size(); ++idx)
    {
        const size_t trackId = vecInliers[idx];
        const track::submapTrack& track = tracks.at(trackId);
        Observations& obs = structure[idx].obs;
        for (track::Track::TrackInfoPerView::const_iterator it = track.begin(); it != track.end(); ++it)
        {
            // get view Id and feat ID
            const size_t viewIndex = it->first;
            const size_t featIndex = it->second;

            // initialize view and get intrinsics
            const View* view = sfmData.getViews().at(viewIndex).get();
            const camera::IntrinsicBase* cam = sfmData.getIntrinsics().find(view->getIntrinsicId())->second.get();
            const camera::Pinhole* intrinsicPtr = dynamic_cast<const camera::Pinhole*>(cam);

            // get normalized feature
            const feature::PointFeature& pt = normalizedFeaturesPerView.getFeatures(viewIndex)[featIndex];
            const Vec2 ptUnnormalized(cam->cam2ima(pt.coords().cast<double>()));
            obs[viewIndex] = Observation(ptUnnormalized, featIndex);
        }
    }

    // Compute 3D landmark positions (triangulation of the observations)
    {
        StructureComputation_blind structureEstimator(false);
        structureEstimator.triangulate(tinyScene);
    }

    // Refine structure and poses (keep intrinsic constant)
    BundleAdjustmentCeres::BA_options options(false, false);
    options._linear_solver_type = ceres::SPARSE_SCHUR;
    BundleAdjustmentCeres bundleAdjustmentObj(options);
    if (bundleAdjustmentObj.Adjust(tiny_scene, REFINE_TRANSLATION | REFINE_STRUCTURE))
    {
        // export scene for visualization
        std::ostringstream os;
        os << posesId.i << "_" << posesId.j << "_" << posesId.k << ".ply";
        save(tinyScene, os.str(), ESfMData(STRUCTURE | EXTRINSICS));

        // Export refined translations
        vecTis[0] = tinyScene.poses[posesId.i].translation();
        vecTis[1] = tinyScene.poses[posesId.j].translation();
        vecTis[2] = tinyScene.poses[posesId.k].translation();
    }
#endif

    // Keep the model iff it has a sufficient inlier count
    const bool bTest = (vecInliers.size() > 30 && 0.33 * tracks.size());

#ifdef DEBUG_TRIPLET
    {
        ALICEVISION_LOG_DEBUG("Triplet : status: " << bTest << " AC: " << dPrecision << " inliers % "
                                                   << double(vecInliers.size()) / tracks.size() * 100.0 << " total putative " << tracks.size());
    }
#endif

    return bTest;
}

}  // namespace sfm
}  // namespace aliceVision
