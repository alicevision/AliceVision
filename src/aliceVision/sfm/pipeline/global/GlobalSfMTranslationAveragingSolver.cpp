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
#include <aliceVision/sfm/BundleAdjustmentCeres.hpp>
#include <aliceVision/sfm/pipeline/global/reindexGlobalSfM.hpp>
#include <aliceVision/sfm/pipeline/global/MutexSet.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/multiview/translationAveraging/common.hpp>
#include <aliceVision/multiview/translationAveraging/solver.hpp>
#include <aliceVision/graph/graph.hpp>
#include <aliceVision/track/TracksBuilder.hpp>
#include <aliceVision/stl/stl.hpp>
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

#include <boost/progress.hpp>

namespace aliceVision {
namespace sfm {

using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfmData;

/// Use features in normalized camera frames
bool GlobalSfMTranslationAveragingSolver::Run(ETranslationAveragingMethod eTranslationAveragingMethod,
                    SfMData& sfmData,
                    const feature::FeaturesPerView& normalizedFeaturesPerView,
                    const matching::PairwiseMatches& pairwiseMatches,
                    const HashMap<IndexT, Mat3>& map_globalR,
                    std::mt19937 & randomNumberGenerator,
                    matching::PairwiseMatches& tripletWise_matches)
{
  // Compute the relative translations and save them to vec_initialRijTijEstimates:
  Compute_translations(sfmData,
        normalizedFeaturesPerView,
        pairwiseMatches,
        map_globalR,
        randomNumberGenerator,
        tripletWise_matches);

  const bool translation = Translation_averaging(eTranslationAveragingMethod, sfmData, map_globalR);

  // Filter matches to keep only them link to view that have valid poses
  // (necessary since multiple components exists before translation averaging)
  std::set<IndexT> valid_view_ids;
  for(const auto & view : sfmData.getViews())
  {
    if(sfmData.isPoseAndIntrinsicDefined(view.second.get()))
      valid_view_ids.insert(view.first);
  }
  KeepOnlyReferencedElement(valid_view_ids, tripletWise_matches);

  return translation;
}

bool GlobalSfMTranslationAveragingSolver::Translation_averaging(ETranslationAveragingMethod eTranslationAveragingMethod,
                      SfMData& sfmData,
                      const HashMap<IndexT, Mat3>& map_globalR)
{
  //-------------------
  //-- GLOBAL TRANSLATIONS ESTIMATION from initial triplets t_ij guess
  //-------------------

  // Keep the largest Biedge connected component graph of relative translations
  PairSet pairs;
  std::transform(m_vec_initialRijTijEstimates.begin(), m_vec_initialRijTijEstimates.end(),
    std::inserter(pairs, pairs.begin()), stl::RetrieveKey());
  const std::set<IndexT> set_remainingIds =
    aliceVision::graph::CleanGraph_KeepLargestBiEdge_Nodes<PairSet, IndexT>(pairs, "./");
  KeepOnlyReferencedElement(set_remainingIds, m_vec_initialRijTijEstimates);

  {
    const std::set<IndexT> index = translationAveraging::getIndexT(m_vec_initialRijTijEstimates);

    const size_t iNview = index.size();
    ALICEVISION_LOG_DEBUG(
      "\n-------------------------------\n"
      " Global translations computation:\n"
      "   - Ready to compute " << iNview << " global translations." << "\n"
      "     from #relative translations: " << m_vec_initialRijTijEstimates.size());

    if(iNview < 3)
    {
      // Too tiny image set to perform motion averaging
      return false;
    }
    //-- Update initial estimates from [minId,maxId] to range [0->Ncam]
    translationAveraging::RelativeInfoVec vec_initialRijTijEstimates_cpy = m_vec_initialRijTijEstimates;
    const PairSet pairs = translationAveraging::getPairs(vec_initialRijTijEstimates_cpy);
    HashMap<IndexT,IndexT> _reindexForward, _reindexBackward;
    reindex(pairs, _reindexForward, _reindexBackward);
    for(size_t i = 0; i < vec_initialRijTijEstimates_cpy.size(); ++i)
    {
      aliceVision::translationAveraging::relativeInfo & rel = vec_initialRijTijEstimates_cpy[i];
      rel.first = Pair(_reindexForward[rel.first.first], _reindexForward[rel.first.second]);
    }

    aliceVision::system::Timer timerLP_translation;

    switch(eTranslationAveragingMethod)
    {
      case TRANSLATION_AVERAGING_L1:
      {
        double gamma = -1.0;
        std::vector<double> vec_solution;
        {
          vec_solution.resize(iNview*3 + vec_initialRijTijEstimates_cpy.size()/3 + 1);
          using namespace aliceVision::linearProgramming;
          #if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
            MOSEKSolver solverLP(vec_solution.size());
          #else
            OSI_CISolverWrapper solverLP(vec_solution.size());
          #endif

          lInfinityCV::Tifromtij_ConstraintBuilder_OneLambdaPerTrif cstBuilder(vec_initialRijTijEstimates_cpy);

          LPConstraintsSparse constraint;
          //-- Setup constraint and solver
          cstBuilder.Build(constraint);
          solverLP.setup(constraint);
          //--
          // Solving
          const bool bFeasible = solverLP.solve();
          ALICEVISION_LOG_DEBUG(" Feasibility " << bFeasible);
          //--
          if (bFeasible)  {
            solverLP.getSolution(vec_solution);
            gamma = vec_solution[vec_solution.size()-1];
          }
          else  {
            ALICEVISION_LOG_WARNING("Compute global translations: failed");
            return false;
          }
        }

        const double timeLP_translation = timerLP_translation.elapsed();
        //-- Export triplet statistics:
        {

          std::ostringstream os;
          os << "Translation fusion statistics.";
          os.str("");
          os << "-------------------------------\n"
            << "-- #relative estimates: " << vec_initialRijTijEstimates_cpy.size()
            << " converge with gamma: " << gamma << ".\n"
            << " timing (s): " << timeLP_translation << ".\n"
            << "-------------------------------" << "\n";
          ALICEVISION_LOG_DEBUG(os.str());
        }

        ALICEVISION_LOG_DEBUG("Found solution:\n" << vec_solution);

        std::vector<double> vec_camTranslation(iNview*3,0);
        std::copy(&vec_solution[0], &vec_solution[iNview*3], &vec_camTranslation[0]);

        std::vector<double> vec_camRelLambdas(&vec_solution[iNview*3], &vec_solution[iNview*3 + vec_initialRijTijEstimates_cpy.size()/3]);

        ALICEVISION_LOG_DEBUG("cam position: " << vec_camTranslation);
        ALICEVISION_LOG_DEBUG("cam Lambdas: " << vec_camRelLambdas);

        // Update the view poses according the found camera centers
        for(std::size_t i = 0; i < iNview; ++i)
        {
          const Vec3 t(vec_camTranslation[i*3], vec_camTranslation[i*3+1], vec_camTranslation[i*3+2]);
          const IndexT pose_id = _reindexBackward[i];
          const Mat3 & Ri = map_globalR.at(pose_id);
          sfmData.setAbsolutePose(pose_id, CameraPose(Pose3(Ri, -Ri.transpose()*t)));
        }
      }
      break;

      case TRANSLATION_AVERAGING_SOFTL1:
      {
        std::vector<Vec3> vec_translations;
        if (!translationAveraging::solve_translations_problem_softl1(
          vec_initialRijTijEstimates_cpy, true, iNview, vec_translations))
        {
          ALICEVISION_LOG_WARNING("Compute global translations: failed");
          return false;
        }

        // A valid solution was found:
        // - Update the view poses according the found camera translations
        for (std::size_t i = 0; i < iNview; ++i)
        {
          const Vec3 & t = vec_translations[i];
          const IndexT pose_id = _reindexBackward[i];
          const Mat3 & Ri = map_globalR.at(pose_id);
          sfmData.setAbsolutePose(pose_id, CameraPose(Pose3(Ri, -Ri.transpose()*t)));
        }
      }
      break;

      case TRANSLATION_AVERAGING_L2_DISTANCE_CHORDAL:
      {
        std::vector<int> vec_edges;
        vec_edges.reserve(vec_initialRijTijEstimates_cpy.size() * 2);
        std::vector<double> vec_poses;
        vec_poses.reserve(vec_initialRijTijEstimates_cpy.size() * 3);
        std::vector<double> vec_weights;
        vec_weights.reserve(vec_initialRijTijEstimates_cpy.size());

        for(int i=0; i < vec_initialRijTijEstimates_cpy.size(); ++i)
        {
          const aliceVision::translationAveraging::relativeInfo & rel = vec_initialRijTijEstimates_cpy[i];
          vec_edges.push_back(rel.first.first);
          vec_edges.push_back(rel.first.second);
          // Since index have been remapped
          // (use the backward indexing to retrieve the second global rotation)
          const IndexT secondId = _reindexBackward[rel.first.second];
          const View * view = sfmData.views.at(secondId).get();
          const Mat3 & Ri = map_globalR.at(view->getPoseId());
          const Vec3 direction = -(Ri.transpose() * rel.second.second.normalized());

          vec_poses.push_back(direction(0));
          vec_poses.push_back(direction(1));
          vec_poses.push_back(direction(2));

          vec_weights.push_back(1.0);
        }

        const double function_tolerance = 1e-7, parameter_tolerance = 1e-8;
        const int max_iterations = 500;

        const double loss_width = 0.0; // No loss in order to compare with TRANSLATION_AVERAGING_L1

        std::vector<double> X(iNview*3, 0.0);
        if(!translationAveraging::solve_translations_problem_l2_chordal(
          &vec_edges[0],
          &vec_poses[0],
          &vec_weights[0],
          vec_initialRijTijEstimates_cpy.size(),
          loss_width,
          &X[0],
          function_tolerance,
          parameter_tolerance,
          max_iterations))  {
            ALICEVISION_LOG_WARNING("Compute global translations: failed");
            return false;
        }

        // Update camera center for each view
        for(std::size_t i = 0; i < iNview; ++i)
        {
          const Vec3 C(X[i*3], X[i*3+1], X[i*3+2]);
          const IndexT pose_id = _reindexBackward[i]; // undo the reindexing
          const Mat3 & Ri = map_globalR.at(pose_id);
          sfmData.setAbsolutePose(pose_id, CameraPose(Pose3(Ri, C)));
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

void GlobalSfMTranslationAveragingSolver::Compute_translations(const SfMData& sfmData,
          const feature::FeaturesPerView & normalizedFeaturesPerView,
          const matching::PairwiseMatches & pairwiseMatches,
          const HashMap<IndexT, Mat3> & map_globalR,
          std::mt19937 & randomNumberGenerator,
          matching::PairwiseMatches & tripletWise_matches)
{
  ALICEVISION_LOG_DEBUG(
    "-------------------------------\n"
    " Relative translations computation:\n"
    "-------------------------------");

  // Compute relative translations over the graph of global rotations
  //  thanks to an edge coverage algorithm
  ComputePutativeTranslation_EdgesCoverage(
    sfmData,
    map_globalR,
    normalizedFeaturesPerView,
    pairwiseMatches,
    randomNumberGenerator,
    m_vec_initialRijTijEstimates,
    tripletWise_matches);
}

//-- Perform a trifocal estimation of the graph contained in vec_triplets with an
// edge coverage algorithm. Its complexity is sub-linear in term of edges count.
void GlobalSfMTranslationAveragingSolver::ComputePutativeTranslation_EdgesCoverage(const SfMData & sfmData,
  const HashMap<IndexT, Mat3> & map_globalR,
  const feature::FeaturesPerView & normalizedFeaturesPerView,
  const matching::PairwiseMatches & pairwiseMatches,
  std::mt19937 & randomNumberGenerator,
  translationAveraging::RelativeInfoVec & vec_initialEstimates,
  matching::PairwiseMatches & newpairMatches)
{
  aliceVision::system::Timer timerLP_triplet;

  //--
  // Compute the relative translations using triplets of rotations over the rotation graph.
  //--
  //
  // 1. List plausible triplets over the global rotation pose graph Ids.
  //   - list all edges that have support in the rotation pose graph
  //
  PairSet rotation_pose_id_graph;
  std::set<IndexT> set_pose_ids;
  std::transform(map_globalR.begin(), map_globalR.end(),
    std::inserter(set_pose_ids, set_pose_ids.begin()), stl::RetrieveKey());
  // List shared correspondences (pairs) between poses
  for (const auto & match_iterator : pairwiseMatches)
  {
    const Pair pair = match_iterator.first;
    const View * v1 = sfmData.getViews().at(pair.first).get();
    const View * v2 = sfmData.getViews().at(pair.second).get();

    if (// Consider the pair iff it is supported by the rotation graph
        (v1->getPoseId() != v2->getPoseId())
        && set_pose_ids.count(v1->getPoseId())
        && set_pose_ids.count(v2->getPoseId()))
    {
      rotation_pose_id_graph.insert(
        std::make_pair(v1->getPoseId(), v2->getPoseId()));
    }
  }
  // List putative triplets (from global rotations Ids)
  const std::vector< graph::Triplet > vec_triplets =
    graph::tripletListing(rotation_pose_id_graph);
  ALICEVISION_LOG_DEBUG("#Triplets: " << vec_triplets.size());

  {
    // Compute triplets of translations
    // Avoid to cover each edge of the graph by using an edge coverage algorithm
    // An estimated triplets of translation mark three edges as estimated.

    //-- precompute the number of track per triplet:
    HashMap<IndexT, IndexT> map_tracksPerTriplets;

    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < (int)vec_triplets.size(); ++i)
    {
      // List matches that belong to the triplet of poses
      const graph::Triplet & triplet = vec_triplets[i];
      matching::PairwiseMatches map_triplet_matches;
      const std::set<IndexT> set_triplet_pose_ids = {triplet.i, triplet.j, triplet.k};
      // List shared correspondences (pairs) between the triplet poses
      for (const auto & match_iterator : pairwiseMatches)
      {
        const Pair pair = match_iterator.first;
        const View * v1 = sfmData.getViews().at(pair.first).get();
        const View * v2 = sfmData.getViews().at(pair.second).get();
        if (// Consider the pair iff it is supported by the triplet graph & 2 different pose id
            (v1->getPoseId() != v2->getPoseId())
            && set_triplet_pose_ids.count(v1->getPoseId())
            && set_triplet_pose_ids.count(v2->getPoseId()))
        {
          map_triplet_matches.insert( match_iterator );
        }
      }
      // Compute tracks:
      {
        aliceVision::track::TracksBuilder tracksBuilder;
        tracksBuilder.build(map_triplet_matches);
        tracksBuilder.filter(true,3);

        #pragma omp critical
        map_tracksPerTriplets[i] = tracksBuilder.nbTracks(); //count the # of matches in the UF tree
      }
    }

    typedef Pair myEdge;

    //-- Alias (list triplet ids used per pose id edges)
    std::map<myEdge, std::vector<size_t> > map_tripletIds_perEdge;
    for (size_t i = 0; i < vec_triplets.size(); ++i)
    {
      const graph::Triplet & triplet = vec_triplets[i];
      map_tripletIds_perEdge[std::make_pair(triplet.i, triplet.j)].push_back(i);
      map_tripletIds_perEdge[std::make_pair(triplet.i, triplet.k)].push_back(i);
      map_tripletIds_perEdge[std::make_pair(triplet.j, triplet.k)].push_back(i);
    }

    // Collect edges that are covered by the triplets
    std::vector<myEdge > vec_edges;
    std::transform(map_tripletIds_perEdge.begin(), map_tripletIds_perEdge.end(), std::back_inserter(vec_edges), stl::RetrieveKey());

    aliceVision::sfm::MutexSet<myEdge> m_mutexSet;

    boost::progress_display my_progress_bar(
      vec_edges.size(),
      std::cout,
      "\nRelative translations computation (edge coverage algorithm)\n");

    // set number of threads, 1 if openMP is not enabled  
    std::vector<translationAveraging::RelativeInfoVec> initial_estimates(omp_get_max_threads());
    const bool bVerbose = false;

    #pragma omp parallel for schedule(dynamic)
    for (int k = 0; k < vec_edges.size(); ++k)
    {
      const myEdge & edge = vec_edges[k];
      #pragma omp critical
      {
        ++my_progress_bar;
      }
      if (m_mutexSet.count(edge) == 0 && m_mutexSet.size() != vec_edges.size())
      {
        // Find the triplets that support the given edge
        const auto & vec_possibleTripletIndexes = map_tripletIds_perEdge.at(edge);

        //-- Sort the triplets according the number of track they are supporting
        std::vector<size_t> vec_commonTracksPerTriplets;
        for (const size_t triplet_index : vec_possibleTripletIndexes)
        {
          vec_commonTracksPerTriplets.push_back(map_tracksPerTriplets[triplet_index]);
        }

        using namespace stl::indexed_sort;
        std::vector< sort_index_packet_descend < size_t, size_t> > packet_vec(vec_commonTracksPerTriplets.size());
        sort_index_helper(packet_vec, &vec_commonTracksPerTriplets[0]);

        std::vector<size_t> vec_triplet_ordered(vec_commonTracksPerTriplets.size(), 0);
        for (size_t i = 0; i < vec_commonTracksPerTriplets.size(); ++i) {
          vec_triplet_ordered[i] = vec_possibleTripletIndexes[packet_vec[i].index];
        }

        // Try to solve a triplet of translations for the given edge
        for (const size_t triplet_index : vec_triplet_ordered)
        {
          const graph::Triplet & triplet = vec_triplets[triplet_index];

          // If the triplet is already estimated by another thread; try the next one
          if (m_mutexSet.count(Pair(triplet.i, triplet.j)) &&
              m_mutexSet.count(Pair(triplet.i, triplet.k)) &&
              m_mutexSet.count(Pair(triplet.j, triplet.k)))
          {
            break;
          }

          //--
          // Try to estimate this triplet of translations
          //--
          double dPrecision = 4.0; // upper bound of the residual pixel reprojection error

          std::vector<Vec3> vec_tis(3);
          std::vector<size_t> vec_inliers;
          aliceVision::track::TracksMap pose_triplet_tracks;

          const std::string sOutDirectory = "./";
          const bool bTriplet_estimation = Estimate_T_triplet(
              sfmData,
              map_globalR,
              normalizedFeaturesPerView,
              pairwiseMatches,
              triplet,
              randomNumberGenerator,
              vec_tis,
              dPrecision,
              vec_inliers,
              pose_triplet_tracks,
              sOutDirectory);

          if (bTriplet_estimation)
          {
            // Since new translation edges have been computed, mark their corresponding edges as estimated
            m_mutexSet.insert(std::make_pair(triplet.i, triplet.j));
            m_mutexSet.insert(std::make_pair(triplet.j, triplet.k));
            m_mutexSet.insert(std::make_pair(triplet.i, triplet.k));

            // Compute the triplet relative motions (IJ, JK, IK)
            {
              const Mat3
                RI = map_globalR.at(triplet.i),
                RJ = map_globalR.at(triplet.j),
                RK = map_globalR.at(triplet.k);
              const Vec3
                ti = vec_tis[0],
                tj = vec_tis[1],
                tk = vec_tis[2];

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
              const int thread_id = omp_get_thread_num();

              initial_estimates[thread_id].emplace_back(
                std::make_pair(triplet.i, triplet.j), std::make_pair(Rij, tij));
              initial_estimates[thread_id].emplace_back(
                std::make_pair(triplet.j, triplet.k), std::make_pair(Rjk, tjk));
              initial_estimates[thread_id].emplace_back(
                std::make_pair(triplet.i, triplet.k), std::make_pair(Rik, tik));

              //--- ATOMIC
              #pragma omp critical
              {
                // Add inliers as valid pairwise matches
                for (std::vector<size_t>::const_iterator iterInliers = vec_inliers.begin();
                  iterInliers != vec_inliers.end(); ++iterInliers)
                {
                  using namespace aliceVision::track;
                  TracksMap::iterator it_tracks = pose_triplet_tracks.begin();
                  std::advance(it_tracks, *iterInliers);
                  const Track & track = it_tracks->second;

                  // create pairwise matches from inlier track
                  for (size_t index_I = 0; index_I < track.featPerView.size() ; ++index_I)
                  {
                    Track::FeatureIdPerView::const_iterator iter_I = track.featPerView.begin();
                    std::advance(iter_I, index_I);

                    // extract camera indexes
                    const size_t id_view_I = iter_I->first;
                    const size_t id_feat_I = iter_I->second;

                    // loop on subtracks
                    for (size_t index_J = index_I+1; index_J < track.featPerView.size() ; ++index_J)
                    {
                      Track::FeatureIdPerView::const_iterator iter_J = track.featPerView.begin();
                      std::advance(iter_J, index_J);

                      // extract camera indexes
                      const size_t id_view_J = iter_J->first;
                      const size_t id_feat_J = iter_J->second;

                      newpairMatches[std::make_pair(id_view_I, id_view_J)][track.descType].emplace_back(id_feat_I, id_feat_J);
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
    for(const auto vec : initial_estimates)
    {
      for(const auto val : vec)
      {
        vec_initialEstimates.emplace_back(val);
      }
    }
  }


  const double timeLP_triplet = timerLP_triplet.elapsed();
  ALICEVISION_LOG_DEBUG("TRIPLET COVERAGE TIMING: " << timeLP_triplet << " seconds");

  ALICEVISION_LOG_DEBUG(
      "-------------------------------\n"
      "-- #Relative translations estimates: " << m_vec_initialRijTijEstimates.size()/3 <<
      " computed from " << vec_triplets.size() << " triplets.\n"
      "-- resulting in " << m_vec_initialRijTijEstimates.size() << " translations estimation.\n"
      "-- timing to obtain the relative translations: " << timeLP_triplet << " seconds.\n"
      "-------------------------------");
}

// Robust estimation and refinement of a translation and 3D points of an image triplets.
bool GlobalSfMTranslationAveragingSolver::Estimate_T_triplet(
  const SfMData& sfmData,
  const HashMap<IndexT, Mat3>& map_globalR,
  const feature::FeaturesPerView& normalizedFeaturesPerView,
  const matching::PairwiseMatches& pairwiseMatches,
  const graph::Triplet& poses_id,
  std::mt19937 & randomNumberGenerator,
  std::vector<Vec3>& vec_tis,
  double& precision, // UpperBound of the precision found by the AContrario estimator
  std::vector<std::size_t>& vec_inliers,
  aliceVision::track::TracksMap& tracks,
  const std::string& outDirectory) const
{
  // List matches that belong to the triplet of poses
  matching::PairwiseMatches map_triplet_matches;
  const std::set<IndexT> set_pose_ids = {poses_id.i, poses_id.j, poses_id.k};
  // List shared correspondences (pairs) between poses
  for (const auto & match_iterator : pairwiseMatches)
  {
    const Pair& pair = match_iterator.first;
    const View* v1 = sfmData.getViews().at(pair.first).get();
    const View* v2 = sfmData.getViews().at(pair.second).get();
    if (// Consider the pair iff it is supported by the triplet graph & 2 different pose id
        (v1->getPoseId() != v2->getPoseId())
        && set_pose_ids.count(v1->getPoseId())
        && set_pose_ids.count(v2->getPoseId()))
    {
      map_triplet_matches.insert( match_iterator );
    }
  }

  aliceVision::track::TracksBuilder tracksBuilder;
  tracksBuilder.build(map_triplet_matches);
  tracksBuilder.filter(true,3);
  tracksBuilder.exportToSTL(tracks);

  if (tracks.size() < 30)
    return false;

  // Convert data
  Mat x1(2, tracks.size());
  Mat x2(2, tracks.size());
  Mat x3(2, tracks.size());

  Mat* xxx[3] = {&x1, &x2, &x3};
  std::set<IndexT> intrinsic_ids;
  size_t cpt = 0;
  for (track::TracksMap::const_iterator iterTracks = tracks.begin();
    iterTracks != tracks.end(); ++iterTracks, ++cpt)
  {
    const track::Track & track = iterTracks->second;
    size_t index = 0;
    for (track::Track::FeatureIdPerView::const_iterator iter = track.featPerView.begin(); iter != track.featPerView.end(); ++iter, ++index)
    {
      const size_t idx_view = iter->first;
      const feature::PointFeature pt = normalizedFeaturesPerView.getFeatures(idx_view, track.descType)[iter->second];
      xxx[index]->col(cpt) = pt.coords().cast<double>();
      const View * view = sfmData.views.at(idx_view).get();
      intrinsic_ids.insert(view->getIntrinsicId());
    }
  }
  // Retrieve the smallest focal value, for threshold normalization
  double min_focal = std::numeric_limits<double>::max();
  for (const auto & ids : intrinsic_ids)
  {
    const camera::IntrinsicBase * intrinsicPtr = sfmData.getIntrinsics().at(ids).get();
    const camera::Pinhole * intrinsic = dynamic_cast< const camera::Pinhole * > (intrinsicPtr);
    if (intrinsic && intrinsic->isValid())
    {
      min_focal = std::min(min_focal, intrinsic->getFocalLengthPixX());
    }
  }
  if (min_focal == std::numeric_limits<double>::max())
  {
    return false;
  }

  // Get rotations:
  const std::vector<Mat3> vec_global_R_Triplet =
    {map_globalR.at(poses_id.i), map_globalR.at(poses_id.j), map_globalR.at(poses_id.k)};

  using namespace aliceVision::trifocal;
  using namespace aliceVision::trifocal::kernel;

  typedef TranslationTripletKernelACRansac<
    translations_Triplet_Solver,
    translations_Triplet_Solver,
    TrifocalTensorModel> KernelType;
  const double ThresholdUpperBound = 1.0e-2; // upper bound of the pixel residual (normalized coordinates)
  KernelType kernel(x1, x2, x3, vec_global_R_Triplet, Mat3::Identity(), ThresholdUpperBound);

  const size_t ORSA_ITER = 320;  // max number of iterations of AC-RANSAC

  TrifocalTensorModel T;
  const std::pair<double,double> acStat =
    robustEstimation::ACRANSAC(kernel, randomNumberGenerator, vec_inliers, ORSA_ITER, &T, precision/min_focal);
  // If robust estimation fails => stop.
  if (precision == std::numeric_limits<double>::infinity())
    return false;

  // Update output parameters
  precision = acStat.first * min_focal;

  vec_tis.resize(3);
  Mat3 K, R;
  KRt_from_P(T.P1, &K, &R, &vec_tis[0]);
  KRt_from_P(T.P2, &K, &R, &vec_tis[1]);
  KRt_from_P(T.P3, &K, &R, &vec_tis[2]);

#ifdef DEBUG_TRIPLET
  // compute 3D scene base on motion estimation
  SfMData tiny_scene;

  // intialize poses (which are now shared by a group of images)
  tiny_scene.poses[poses_id.i] = Pose3(vec_global_R_Triplet[0], -vec_global_R_Triplet[0].transpose() * vec_tis[0]);
  tiny_scene.poses[poses_id.j] = Pose3(vec_global_R_Triplet[1], -vec_global_R_Triplet[1].transpose() * vec_tis[1]);
  tiny_scene.poses[poses_id.k] = Pose3(vec_global_R_Triplet[2], -vec_global_R_Triplet[2].transpose() * vec_tis[2]);

  // insert views used by the relative pose pairs
  for (const auto & pairIterator : map_triplet_matches )
  {
    // initialize camera indexes
    const IndexT I = pairIterator.first.first;
    const IndexT J = pairIterator.first.second;

    // add views
    tiny_scene.views.insert(*sfm_data.getViews().find(I));
    tiny_scene.views.insert(*sfm_data.getViews().find(J));

    // add intrinsics
    const View * view_I = sfm_data.getViews().at(I).get();
    const View * view_J = sfm_data.getViews().at(J).get();
    tiny_scene.intrinsics.insert(*sfm_data.getIntrinsics().find(view_I->getIntrinsicId()));
    tiny_scene.intrinsics.insert(*sfm_data.getIntrinsics().find(view_J->getIntrinsicId()));
  }

  // Fill sfm_data with the inliers tracks. Feed image observations: no 3D yet.
  Landmarks & structure = tiny_scene.structure;
  for(size_t idx=0; idx < vec_inliers.size(); ++idx)
  {
    const size_t trackId = vec_inliers[idx];
    const track::submapTrack & track = tracks.at(trackId);
    Observations & obs = structure[idx].obs;
    for (track::Track::FeatureIdPerView::const_iterator it = track.begin(); it != track.end(); ++it)
    {
      // get view Id and feat ID
      const size_t viewIndex = it->first;
      const size_t featIndex = it->second;

      // initialize view and get intrinsics
      const View * view = sfm_data.getViews().at(viewIndex).get();
      const camera::IntrinsicBase *  cam = sfm_data.getIntrinsics().find(view->getIntrinsicId())->second.get();
      const camera::Pinhole * intrinsicPtr = dynamic_cast< const camera::Pinhole * >(cam);

      // get normalized feature
      const feature::PointFeature & pt = normalizedFeaturesPerView.getFeatures(viewIndex)[featIndex];
      const Vec2 pt_unnormalized (cam->cam2ima(pt.coords().cast<double>()));
      obs[viewIndex] = Observation(pt_unnormalized, featIndex);
    }
  }

  // Compute 3D landmark positions (triangulation of the observations)
  {
    StructureComputation_blind structure_estimator(false);
    structure_estimator.triangulate(tiny_scene);
  }

  // Refine structure and poses (keep intrinsic constant)
  BundleAdjustmentCeres::BA_options options(false, false);
  options._linear_solver_type = ceres::SPARSE_SCHUR;
  BundleAdjustmentCeres bundle_adjustment_obj(options);
  if (bundle_adjustment_obj.Adjust(tiny_scene, REFINE_TRANSLATION | REFINE_STRUCTURE))
  {
    // export scene for visualization
    std::ostringstream os;
    os << poses_id.i << "_" << poses_id.j << "_" << poses_id.k << ".ply";
    Save(tiny_scene, os.str(), ESfMData(STRUCTURE | EXTRINSICS));

    // Export refined translations
    vec_tis[0] = tiny_scene.poses[poses_id.i].translation();
    vec_tis[1] = tiny_scene.poses[poses_id.j].translation();
    vec_tis[2] = tiny_scene.poses[poses_id.k].translation();
  }
#endif

  // Keep the model iff it has a sufficient inlier count
  const bool bTest = ( vec_inliers.size() > 30 && 0.33 * tracks.size() );

#ifdef DEBUG_TRIPLET
  {
    ALICEVISION_LOG_DEBUG(
      "Triplet : status: " << bTest <<
      " AC: " << dPrecision <<
      " inliers % " << double(vec_inliers.size()) / tracks.size() * 100.0 <<
      " total putative " << tracks.size());
  }
#endif

  return bTest;
}

} // namespace sfm
} // namespace aliceVision

