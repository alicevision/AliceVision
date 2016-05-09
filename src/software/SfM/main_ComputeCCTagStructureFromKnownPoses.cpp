
// Copyright (c) 2015 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include "openMVG/sfm/sfm.hpp"
#include "openMVG/system/timer.hpp"
#include "openMVG/features/svgVisualization.hpp"

#include "openMVG/features/cctag/CCTAG_describer.hpp"

#include "boost/filesystem.hpp"
#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

using namespace openMVG;
using namespace openMVG::sfm;
namespace bfs = boost::filesystem;

void alignAndScaleStructure(
    SfM_Data & sfmData,
    const IndexT nCCTags,
    const IndexT indexOrigin,
    const IndexT indexXDirection)
{
  const std::size_t nLandmarks = sfmData.GetLandmarks().size();
  Mat3X vX(3,nLandmarks);
  std::size_t i=0;
  for(const auto & landmark : sfmData.GetLandmarks())
  {
    vX.col(i) = landmark.second.X;
    ++i;
  }
  // Compute the mean of the point cloud
  Vec3 meanPoints = Vec3::Zero(3,1);
  for(int i=0 ; i < nCCTags ; ++i )
  {
    meanPoints +=  vX.col(i);
  }
  meanPoints /= nCCTags;
  
  // Center the point cloud in [0;0;0]
  for(int i=0 ; i < nLandmarks ; ++i )
  {
    vX.col(i) -= meanPoints;
  }
  
  const std::size_t nCameras = sfmData.GetPoses().size();
  Mat3X vCamCenter(3,nCameras);
  std::vector<Mat3> vCamRotation; // Camera rotations
  vCamRotation.reserve(nCameras);
  i=0;
  for(const auto & pose : sfmData.GetPoses())
  {
    vCamCenter.col(i) = pose.second.center() - meanPoints;
    vCamRotation.push_back(pose.second.rotation().transpose()); // Rotation in the world coordinate system
    ++i;
  }
  
  // Perform an svd over vX*vXT (var-covar)
  Mat3 dum = vX.leftCols(nCCTags) * vX.leftCols(nCCTags).transpose();
  Eigen::JacobiSVD<Mat3> svd(dum,Eigen::ComputeFullV|Eigen::ComputeFullU);
  Mat3 U = svd.matrixU();
  
  // Check whether the determinant is negative in order to keep
  // a direct coordinate system
  if ( U.determinant() < 0 )
  {
    U.col(2) = -U.col(2);
  }
  
  for(int i=0 ; i < nLandmarks ; ++i )
  {
    vX.col(i) = U.transpose() * vX.col(i);
  }

  for(int i=0 ; i < nCameras ; ++i )
  {
    vCamCenter.col(i) = U.transpose() * vCamCenter.col(i);
    vCamRotation[i] = U.transpose() * vCamRotation[i];
  }
  
  // Compute the maximal distance between the two farthest points
  double dMax = 0;
  std::size_t iMax = 0;
  std::size_t jMax = 0;
  for(int i=0 ; i < nCCTags ; ++i )
  {
    for(int j=0 ; j < nCCTags ; ++j )
    {
      double dP1P2 = Vec3(vX.col(i) - vX.col(j)).norm();
      if ( dP1P2 > dMax)
      {
        dMax = dP1P2;
        iMax = i;
        jMax = j;
      }
    }
  }
  
  // Parameter 1 to set
  const double farthestDistance = 2*sqrt(2);
  const double scaleFactor = farthestDistance/dMax;
  
  // Point cloud scaling
  vX *= scaleFactor;
  vCamCenter *= scaleFactor;
  
  // Cameras used for the acquisition are located below the markers/ceiling
  // Therefore, z-camera centers are supposed to be negative, the point cloud
  // being centered in (0,0,0), the z axis pointing up.
  Mat3 flip = Eigen::MatrixXd::Identity(3,3);
  if ( vCamCenter.col(0)[2] < 0 ) // Markers on the ceiling: > 0, on the ground < 0
  {
    flip(0,0) = -flip(0,0);
    flip(2,2) = -flip(2,2);
    
    for(int i=0 ; i < nLandmarks ; ++i )
      vX.col(i) = flip * vX.col(i);
    
    for(int i=0 ; i < nCameras ; ++i )
    {
      vCamCenter.col(i) = flip * vCamCenter.col(i);
      vCamRotation[i] = flip * vCamRotation[i];
    }
  }
  
//  // Point cloud translation along the z-axis if needed
//  const double height = 5;
//  // Center the point cloud in [0;0;0]
//  for(int i=0 ; i < nLandmarks ; ++i )
//  {
//    vX.col(i) += Vec3(0,0,height);
//    
//  }
  
  Vec3 origin = Vec3::Zero(3,1);
  //if (indexOrigin == UndefinedIndexT)
  //{
  //  // Centering based on the middle of the diagonal
  //  origin = Vec3(vX.col(iMax) + vX.col(jMax))/2;
  //}
  //else
  //{
    origin = Vec3(vX.col(indexOrigin));
  //}
  for(int i=0 ; i < nLandmarks ; ++i )
    vX.col(i) -= origin;
    
  for(int i=0 ; i < nCameras ; ++i )
    vCamCenter.col(i) -= origin; 
  
  // 2D rotation to align to the square [-xmin -ymin +xmax +ymax]
  double thetaAlign = -atan2(vX(1,indexXDirection),vX(0,indexXDirection));// + M_PI/4;
  //double thetaAlign = -atan2(vX(1,iMax),vX(0,iMax)) + M_PI/4;
  double cosAlign = cos(thetaAlign);
  double sinAlign = sin(thetaAlign);

  Mat3 alignSquareRot = Eigen::MatrixXd::Identity(3,3);
  alignSquareRot(0,0) = cosAlign;
  alignSquareRot(0,1) = -sinAlign;
  alignSquareRot(1,0) = sinAlign;
  alignSquareRot(1,1) = cosAlign;
  
  for(int i=0 ; i < nLandmarks ; ++i )
    vX.col(i) = alignSquareRot * vX.col(i);
    
  for(int i=0 ; i < nCameras ; ++i )
  {
    vCamCenter.col(i) = alignSquareRot * vCamCenter.col(i);
    vCamRotation[i] = alignSquareRot * vCamRotation[i];
  }
  
  i=0;
  // Matlab style output for debug
  //std::cout << " [ " ;
  
  // Write back the transformed structure
  for( auto & landmark : sfmData.structure )
  {
    landmark.second.X = vX.col(i);
    //std::cout << " [ " << landmark.second.X(0) << " " << landmark.second.X(1) << " " << landmark.second.X(2)<< " ] ; ";
    ++i;
  }
  //std::cout << " ] " ;
  
  i=0;
  // Write back the transformed cameras
  for( auto & pose : sfmData.poses )
  {
    pose.second = geometry::Pose3(vCamRotation[i].transpose(), vCamCenter.col(i));
    ++i;
  }
}

std::ostream& operator<<(std::ostream& stream, const std::set<IndexT>& s)
{
  stream << "{";
  for(IndexT i: s)
    stream << i << ", ";
  stream << "}" << std::endl;
  return stream;
}

/// Compute the structure of a scene according existing camera poses.
int main(int argc, char **argv)
{
  using namespace std;
  std::cout << "Compute CCTag Structure from the provided poses" << std::endl;

  CmdLine cmd;
  std::string sSfM_Data_Filename;
  std::string sMatchesDir;
  std::string sOutFile;
  std::string sDebugOutputDir;
  bool sUseSfmVisibility = false;
  bool sKeepSift = false;
  IndexT idOrigin = UndefinedIndexT;
  IndexT idXDirection = UndefinedIndexT;
  IndexT minTrackLength = 4;

  cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
  cmd.add( make_option('m', sMatchesDir, "match_dir") );
  cmd.add( make_option('o', sOutFile, "output_file") );
  cmd.add( make_option('s', sKeepSift, "keep_sift") );
  cmd.add( make_option('r', sUseSfmVisibility, "use_sfm_visibility") );
  cmd.add( make_option('d', sDebugOutputDir, "debug_dir") );
  cmd.add( make_option('c', idOrigin, "cctag_id_origin") );
  cmd.add( make_option('x', idXDirection, "cctag_id_x_direction") );
  cmd.add( make_option('t', minTrackLength, "min_track_length") );

  try {
    if (argc == 1) throw std::string("Invalid command line parameter.");
    cmd.process(argc, argv);
  } catch(const std::string& s) {
    std::cerr << "Usage: " << argv[0] << '\n'
    << "[-i|--input_file] path to a SfM_Data scene\n"
    << "[-m|--match_dir] path to the features and descriptor that "
    << " corresponds to the provided SfM_Data scene\n"
    << "[-f|--match_file] (opt.) path to a matches file (used pairs will be used)\n"
    << "[-o|--output_file] file where the output data will be stored\n"
    << "[-s|--keep_sift] keep SIFT points (default false)\n"
    << "[-r|--use_sfm_visibility] Use connections between views based on SfM observations instead of relying on frustums intersections (default false)\n"
    << "[-d|--debug_dir] debug output directory to generate svg files with detected CCTags (default \"\")\n"
    << "[-c|--cctag_id_origin] id of the cctag defining the origin of the coordinate system (if not set, the cctags gravity center will be used)\n"
    << "[-x|--cctag_id_x_direction] id of the cctag defining the x direction of the coordinate system (if not set, dominant direction of the point cloud will be used)\n"
    << "[-t|--min_track_length] minimum of the track lengths required by a point to be triangulated (default "<< minTrackLength << ")\n"     
    << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }

  // Load input SfM_Data scene
  SfM_Data reconstructionSfmData;
  if (!Load(reconstructionSfmData, sSfM_Data_Filename, ESfM_Data::ALL)) {
    std::cerr << std::endl
      << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  // Init the regions_type from the image describer file (used for image regions extraction)
  using namespace openMVG::features;
  const std::string sImage_describer = stlplus::create_filespec(sMatchesDir, "image_describer", "json");
  std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
  if (!regions_type)
  {
    std::cerr << "Invalid: "
      << sImage_describer << " regions type file." << std::endl;
    return EXIT_FAILURE;
  }

  // Prepare the Regions provider
  std::shared_ptr<Regions_Provider> regions_provider = std::make_shared<Regions_Provider>();
  if (!regions_provider->load(reconstructionSfmData, sMatchesDir, regions_type)) {
    std::cerr << std::endl
      << "Invalid regions." << std::endl;
    return EXIT_FAILURE;
  }

  //--
  //- Pair selection method
  //  - geometry guided -> camera frustum intersection
  //  - putative matches guided (photometric matches) (Keep pair that have valid Intrinsic & Pose ids)
  //--
  std::cout << "Compute connected views by frustrum intersection." << std::endl;
  std::map<IndexT, std::set<IndexT>> connectedViews;
  {
    Pair_Set viewPairs;
    if (sUseSfmVisibility)
    {
      PairWiseMatches matches;
      if (!matching::Load(matches, reconstructionSfmData.GetViewsKeys(), sMatchesDir, "f"))
      {
        std::cerr<< "Unable to read the matches file." << std::endl;
        return EXIT_FAILURE;
      }
      viewPairs = getPairs(matches);

      // Keep only Pairs that belong to valid view indexes.
      viewPairs = Pair_filter(viewPairs, Get_Valid_Views(reconstructionSfmData));
    }
    else
    {
      // No image pair provided, so we use cameras frustum intersection.
      // Build the list of connected images pairs from frustum intersections
      viewPairs = Frustum_Filter(reconstructionSfmData).getFrustumIntersectionPairs();
    }
    
    // Convert pair to map
    for(auto& p: viewPairs)
    {
      connectedViews[p.first].insert(p.second);
      connectedViews[p.second].insert(p.first);
    }
  }
  
  std::cout << "Database of all CCTags" << std::endl;
  // Database of all CCTags: <CCTagId, set<ViewID>>
  std::map<IndexT, std::set<IndexT>> cctagsVisibility;
  // Database of all CCTag observations: <(CCTagId, ViewID), Observation>
  std::map<std::pair<IndexT, IndexT>, Observation> cctagsObservations;
  {
    // List all CCTags in descriptors of all reconstructed cameras
    for(const auto& regionForView: regions_provider->regions_per_view)
    {
      View* view = reconstructionSfmData.GetViews().at(regionForView.first).get();
      if (!reconstructionSfmData.IsPoseAndIntrinsicDefined(view))
      {
        // Consider only reconstructed cameras
        std::cout << "Ignore unreconstructed view (viewId: " << view->id_view << ", poseId: " << view->id_pose << ")" << std::endl;
        continue;
      }
      const features::Regions* regions = regionForView.second.get();
      const features::SIFT_Regions* siftRegions = dynamic_cast<const features::SIFT_Regions*>(regions);
      if(siftRegions == nullptr)
      {
        throw std::runtime_error("Only works with SIFT regions in input.");
      }
      features::SIFT_Regions cctagRegions_debug;
      for(std::size_t i = 0; i < siftRegions->RegionCount(); ++i)
      {
        const features::SIFT_Regions::DescriptorT& cctagDesc = siftRegions->Descriptors()[i];
        IndexT cctagId = features::getCCTagId<features::SIFT_Regions::DescriptorT>(cctagDesc);
        
        if(cctagId == UndefinedIndexT)
          // Not a CCTag
          continue;

        cctagsVisibility[cctagId].insert(regionForView.first);
        cctagsObservations[std::make_pair(cctagId, regionForView.first)] = Observation(siftRegions->Features()[i].coords().cast<double>(), i);
        
        if(!sDebugOutputDir.empty())
        {
          cctagRegions_debug.Features().push_back(siftRegions->Features()[i]);
          cctagRegions_debug.Descriptors().push_back(cctagDesc);
        }
      }
      // DEBUG: export svg files
      if(!sDebugOutputDir.empty())
      {
        cameras::IntrinsicBase* intrinsics = reconstructionSfmData.GetIntrinsics().at(view->id_intrinsic).get();
        localization::saveCCTag2SVG(view->s_Img_path, 
                std::make_pair(intrinsics->w(), intrinsics->h()),
                cctagRegions_debug,
                (bfs::path(sDebugOutputDir) / bfs::path(bfs::path(view->s_Img_path).stem().string()+".svg")).string());
      }
    }
  }

  std::cout << "Convert list of all CCTag into landmarks" << std::endl;
  // Convert list of all CCTag into landmarks.
  // The same CCTag ID could be used at different places, so we check frustum intersection
  // to determine the group of CCTag visibility.
  SfM_Data cctagSfmData;
  cctagSfmData.views = reconstructionSfmData.views;
  cctagSfmData.intrinsics = reconstructionSfmData.intrinsics;
  cctagSfmData.poses = reconstructionSfmData.poses;

  IndexT landmarkMaxIndex = 0;
  if(sKeepSift)
  {
    // Ensure we will not reuse the same landmark ID
    for(const auto& landmarkById: reconstructionSfmData.GetLandmarks())
    {
      landmarkMaxIndex = std::max(landmarkById.first+1, landmarkMaxIndex);
    }
  }

  IndexT landmarkIndex = landmarkMaxIndex;
  
  IndexT indexLandmarkOrigin = UndefinedIndexT;
  bool flagOrigin = false;
  
  IndexT indexLandmarkXDirection = UndefinedIndexT;
  bool flagXDirection = false;
  
  for(const auto& cctagVisibility: cctagsVisibility)
  {
    std::set<IndexT> viewsWithSameCCTagId = cctagVisibility.second;
    // split groups of cctags with the same ID
    while(!viewsWithSameCCTagId.empty())
    {
      IndexT obsViewId = *viewsWithSameCCTagId.begin();

      std::set<IndexT> cctagSubGroup;
      cctagSubGroup.insert(obsViewId);
      // cctagSubGroup = intersection(connectedViews[obsViewId], cctagsWithSameId)
      std::set_intersection(
        connectedViews[obsViewId].begin(), connectedViews[obsViewId].end(),
        viewsWithSameCCTagId.begin(), viewsWithSameCCTagId.end(),
        std::inserter(cctagSubGroup, cctagSubGroup.end()));
      
      if(cctagSubGroup.size() >= minTrackLength)
      {
        // Create a new landmark for this CCTag subgroup
        Landmark& landmark = cctagSfmData.structure[landmarkIndex];
        // landmark.X; keep default value, will be set by triangulation in the next step.
        for(IndexT iObsViewId: cctagSubGroup)
        {
          landmark.obs[iObsViewId] = cctagsObservations[std::make_pair(cctagVisibility.first, iObsViewId)];
        }
        
        if (idOrigin == cctagVisibility.first)
        {
          if (!flagOrigin)
          {
            flagOrigin = true;
            indexLandmarkOrigin = landmarkIndex;
          }else
          {
            std::cout << " WARNING: the id of the cctag origin is associated to a cctag existing at least two times in the reconstruction." << std::endl;
          }
        }
        
        if (idXDirection == cctagVisibility.first)
        {
          if (!flagXDirection)
          {
            flagXDirection = true;
            indexLandmarkXDirection = landmarkIndex;
          }else
          {
            std::cout << " WARNING: the id of the cctag defining the X direction is associated to a cctag existing at least two times in the reconstruction." << std::endl;
          }
        }
        
        ++landmarkIndex;
      }
      // Remove the subgroup from the main one
      for(IndexT i: cctagSubGroup)
        viewsWithSameCCTagId.erase(i);
    }
  }
  
  // If the id origin has been specified and its associated id has not been found in the views, then the cctag origin is missing.
  if ( (idOrigin != UndefinedIndexT) && (!flagOrigin) )
  {
    std::cout << " WARNING: the id of the cctag origin is associated to a cctag that does not appear enough times in the image collection." << std::endl;
    indexLandmarkOrigin = UndefinedIndexT;
  }
  
  // If the id origin has been specified and its associated id has not been found in the views, then the cctag origin is missing.
  if ( (idXDirection != UndefinedIndexT) && (!flagXDirection) )
  {
    std::cout << " WARNING: the id of the cctag defining the X direction is associated to a cctag that does not appear enough times in the image collection." << std::endl;
    indexLandmarkXDirection = UndefinedIndexT;
  }
  
  openMVG::system::Timer timer;

  //------------------------------------------
  // Compute Structure from known camera poses
  //------------------------------------------
  std::cout << "Compute Structure from known camera poses" << std::endl;

  std::cout << "#CCTag nb input IDs used: " << cctagsVisibility.size() << std::endl;
  const std::size_t cctagLandmarkCandidatesSize = cctagSfmData.structure.size();
  std::cout << "#CCTag landmark candidates: " << cctagLandmarkCandidatesSize << std::endl;
  // Triangulate them using a blind triangulation scheme
  SfM_Data_Structure_Computation_Robust structure_estimator(true);
  structure_estimator.triangulate(cctagSfmData);
  const std::size_t cctagReconstructedLandmarksSize = cctagSfmData.structure.size();
  std::cout << "#CCTag landmark reconstructed: " << cctagReconstructedLandmarksSize << std::endl;
  RemoveOutliers_AngleError(cctagSfmData, 2.0);
  std::cout << "#CCTag landmark found: " << cctagSfmData.GetLandmarks().size() << std::endl;

  std::cout << "\nCCTag Structure estimation took (s): " << timer.elapsed() << "." << std::endl;
  
  if(sKeepSift)
  {
    // Copy non-CCTag landmarks
    for(const auto& landmarkById: reconstructionSfmData.GetLandmarks())
    {
      const auto firstObs = landmarkById.second.obs.cbegin();
      const features::Regions* regions = regions_provider->regions_per_view[firstObs->first].get();
      const features::SIFT_Regions* siftRegions = dynamic_cast<const features::SIFT_Regions*>(regions);
      if(siftRegions == nullptr)
      {
        throw std::runtime_error("Only works with SIFT regions in input.");
      }

      const features::SIFT_Regions::DescriptorT& desc = siftRegions->Descriptors()[firstObs->second.id_feat];
      IndexT cctagId = features::getCCTagId<features::SIFT_Regions::DescriptorT>(desc);
      if(cctagId != UndefinedIndexT)
        // It's a CCTag, so we ignore it and use the newly triangulated CCTags.
        continue;

      // std::cout << "Add SIFT landmark " << landmarkById.first << std::endl;
      cctagSfmData.structure[landmarkById.first] = landmarkById.second;
    }
  }
  
  // Set the coordinate system so that the (x,y) plane is aligned to the plane formed by a set of cctags.
  // Currently all the cctags are considered in the plane fitting.
  // If passed in the command line, the origin of the system correspond to the cctag whose id is idOrigin,
  // otherwise set to the gravity center of all tags.
  alignAndScaleStructure(cctagSfmData, cctagReconstructedLandmarksSize, indexLandmarkOrigin, indexLandmarkXDirection);
  
  if (stlplus::extension_part(sOutFile) != "ply")
  {
    Save(cctagSfmData,
      stlplus::create_filespec(
        stlplus::folder_part(sOutFile),
        stlplus::basename_part(sOutFile), "ply"),
      ESfM_Data(ALL));
  }

  if (Save(cctagSfmData, sOutFile, ESfM_Data(ALL)))
    return EXIT_SUCCESS;
  return EXIT_FAILURE;
}
