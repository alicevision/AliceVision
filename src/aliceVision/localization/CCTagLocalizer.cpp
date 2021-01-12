// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "CCTagLocalizer.hpp"
#include "reconstructed_regions.hpp"
#include "optimization.hpp"
#include "rigResection.hpp"

#include <aliceVision/config.hpp>
#include <aliceVision/matching/svgVisualization.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/sfm/pipeline/RelativePoseInfo.hpp>

#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>

#include <cctag/ICCTag.hpp>

#include <boost/progress.hpp>
#include <boost/filesystem.hpp>

#include <algorithm>
#include <sstream>

namespace aliceVision {
namespace localization {

CCTagLocalizer::CCTagLocalizer(const sfmData::SfMData &sfmData,
                               const std::string &descriptorsFolder)
    : _cudaPipe( 0 )
{
  _sfm_data = sfmData;

  bool loadSuccessful = loadReconstructionDescriptors(_sfm_data, descriptorsFolder);
  
  if(!loadSuccessful)
  {
    ALICEVISION_LOG_ERROR("Unable to load the descriptors");
    throw std::invalid_argument("Unable to load the descriptors from "+descriptorsFolder);
  }
  
//  for(const auto & landmark : landmarks)
//  {
//    // Use the first observation to retrieve the associated descriptor.
//    const auto & firstObservation = *landmark.second.obs.begin();
//    
//    // Retrieve the Regions of the first observation
//    auto & reconstructedRegions = _regions_per_view[firstObservation.first];
//    
//    // Get the feature id: remap the index as we only load the reconstructed regions
//    const auto localFeatureId = reconstructedRegions._mapFullToLocal[firstObservation.second.id_feat];
//    
//    const auto & desc = reconstructedRegions._regions.Descriptors()[localFeatureId];
//    IndexT idCCTag = getCCTagId(desc);
//
//    // Insert <idCCTag, 3D point> into a map.
//    if (idCCTag!=UndefinedIndexT)
//    {
//      _cctagDatabase.emplace(idCCTag, landmark.second.X);
//    }
//  }
  
  _isInit = true;
}


bool CCTagLocalizer::loadReconstructionDescriptors(const sfmData::SfMData & sfm_data,
                                                   const std::string & feat_directory)
{
  ALICEVISION_LOG_DEBUG("Build observations per view");

  // Build observations per view
  std::map<IndexT, std::map<feature::EImageDescriberType, std::vector<feature::FeatureInImage>>> observationsPerView;
  for(const auto& landmarkValue : _sfm_data.structure)
  {
    IndexT trackId = landmarkValue.first;
    const sfmData::Landmark& landmark = landmarkValue.second;

    for(const auto& obs : landmark.observations)
    {
      const IndexT viewId = obs.first;
      const sfmData::Observation& obs2d = obs.second;
      observationsPerView[viewId][landmark.descType].emplace_back(obs2d.id_feat, trackId);
    }
  }
  for(auto& featuresPerTypeInImage : observationsPerView)
  {
    for(auto& featuresInImage : featuresPerTypeInImage.second)
    {
      std::sort(featuresInImage.second.begin(), featuresInImage.second.end());
    }
  }

  ALICEVISION_LOG_DEBUG("Load Features and Descriptors per view");

  std::vector<std::string> featuresFolders = _sfm_data.getFeaturesFolders();
  featuresFolders.emplace_back(feat_directory);

  // Read for each view the corresponding Regions and store them
  for(const auto &iter : _sfm_data.getViews())
  {
    const IndexT id_view = iter.second->getViewId();
    if(observationsPerView.count(id_view) == 0)
      continue;
    const auto& observations = observationsPerView.at(id_view);
    {
      const feature::EImageDescriberType descType = _imageDescriber.getDescriberType();

      if(observations.count(descType) == 0)
      {
        // no descriptor of this type reconstructed in this View
        _imageDescriber.allocate(_regionsPerView.getData()[id_view][descType]);
        continue;
      }

      // Load from files
      std::unique_ptr<feature::Regions> currRegions = sfm::loadRegions(featuresFolders, id_view, _imageDescriber);

      // Filter descriptors to keep only the 3D reconstructed points
      _regionsPerView.getData()[id_view][descType] = createFilteredRegions(*currRegions, observations.at(descType), _reconstructedRegionsMappingPerView[id_view][descType]);
    }
  }


  {
    std::set<int> presentCCtagIds;
    std::vector<int> counterCCtagsInImage = {0, 0, 0, 0, 0, 0};
    // just debugging stuff -- print for each image the visible reconstructed cctag
    // and create an histogram of cctags per image
    for(const auto &iter : sfm_data.getViews())
    {
      const IndexT id_view = iter.second->getViewId();
      auto itViewRegions = _regionsPerView.getData().find(id_view);
      if(itViewRegions == _regionsPerView.getData().end())
        continue;
      auto itViewRegion = itViewRegions->second.find(_cctagDescType);
      if(itViewRegion == itViewRegions->second.end())
        continue;
      const feature::Regions& regions = *itViewRegion->second;
      const std::string &sImageName = iter.second.get()->getImagePath();
      std::stringstream ss;

      ss << "Image " << sImageName;
      if(regions.RegionCount() == 0 )
      {
        counterCCtagsInImage[0] += 1;
        ss << " does not contain any cctag!!!";
      }
      else
      {
        ss << " contains CCTag Id: ";
        const feature::CCTAG_Regions& cctagRegions = dynamic_cast<const feature::CCTAG_Regions&>(regions);
        for(const auto &desc : cctagRegions.Descriptors())
        {
          const IndexT cctagIdA = feature::getCCTagId(desc);
          if(cctagIdA != UndefinedIndexT)
          {
            presentCCtagIds.insert(cctagIdA);
            ss << cctagIdA << " ";
          }
        }
        // Update histogram
        int countcctag = cctagRegions.Descriptors().size();
        if(countcctag >= 5)
          counterCCtagsInImage[5] +=1;
        else
          counterCCtagsInImage[countcctag] += 1;
      }
      ALICEVISION_LOG_DEBUG(ss.str());
    }
    
    // Display histogram
    ALICEVISION_LOG_DEBUG("Histogram of number of cctags in images :");
    for(std::size_t i = 0; i < 5; i++)
      ALICEVISION_LOG_DEBUG("Images with " << i << "  CCTags : " << counterCCtagsInImage[i]);
    ALICEVISION_LOG_DEBUG("Images with 5+ CCTags : " << counterCCtagsInImage[5]);

    // Display the cctag ids over all cctag landmarks present in the database
    ALICEVISION_LOG_DEBUG("Found " << presentCCtagIds.size() << " different CCTag ids in the database with " << _sfm_data.getLandmarks().size() << " associated 3D points\n"
            "The CCTag ids in the database are: ");
    for(int cctagId: presentCCtagIds)
    {
      ALICEVISION_LOG_DEBUG(cctagId);
    }
  }

  return true;
}

bool CCTagLocalizer::localize(const image::Image<float> & imageGrey,
                              const LocalizerParameters *parameters,
                              std::mt19937 & randomNumberGenerator,
                              bool useInputIntrinsics,
                              camera::PinholeRadialK3 &queryIntrinsics,
                              LocalizationResult & localizationResult, 
                              const std::string& imagePath)
{
  namespace bfs = boost::filesystem;
  
  const CCTagLocalizer::Parameters *param = static_cast<const CCTagLocalizer::Parameters *>(parameters);
  if(!param)
  {
    throw std::invalid_argument("The CCTag localizer parameters are not in the right format.");
  }
  // extract descriptors and features from image
  ALICEVISION_LOG_DEBUG("[features]\tExtract CCTag from query image");

  image::Image<unsigned char> imageGrayUChar; // cctag image describer don't support float image
  imageGrayUChar = (imageGrey.GetMat() * 255.f).cast<unsigned char>();

  feature::MapRegionsPerDesc tmpQueryRegions;

  _imageDescriber.setCudaPipe( _cudaPipe );
  _imageDescriber.setConfigurationPreset(param->_featurePreset);
  _imageDescriber.describe(imageGrayUChar, tmpQueryRegions[_cctagDescType]);
  ALICEVISION_LOG_DEBUG("[features]\tExtract CCTAG done: found " << tmpQueryRegions.at(_cctagDescType)->RegionCount() << " features");
  
  std::pair<std::size_t, std::size_t> imageSize = std::make_pair(imageGrey.Width(),imageGrey.Height());
  
  if(!param->_visualDebug.empty() && !imagePath.empty())
  {
    // it automatically throws an exception if the cast does not work
    const feature::CCTAG_Regions & cctagQueryRegions = tmpQueryRegions.getRegions<feature::CCTAG_Regions>(_cctagDescType);
    
    // just debugging -- save the svg image with detected cctag
    matching::saveCCTag2SVG(imagePath, 
                            imageSize, 
                            cctagQueryRegions,
                            param->_visualDebug+"/"+bfs::path(imagePath).stem().string()+".svg");
  }
  return localize(tmpQueryRegions,
                  imageSize,
                  parameters,
                  randomNumberGenerator,
                  useInputIntrinsics,
                  queryIntrinsics,
                  localizationResult,
                  imagePath);
}

void CCTagLocalizer::setCudaPipe( int i )
{
    _cudaPipe = i;
}

bool CCTagLocalizer::localize(const feature::MapRegionsPerDesc & genQueryRegions,
                              const std::pair<std::size_t, std::size_t> &imageSize,
                              const LocalizerParameters *parameters,
                              std::mt19937 & randomNumberGenerator,
                              bool useInputIntrinsics,
                              camera::PinholeRadialK3 &queryIntrinsics,
                              LocalizationResult & localizationResult,
                              const std::string& imagePath)
{
  namespace bfs = boost::filesystem;

  const CCTagLocalizer::Parameters *param = dynamic_cast<const CCTagLocalizer::Parameters *>(parameters);
  if(!param)
  {
    throw std::invalid_argument("The CCTag localizer parameters are not in the right format.");
  }
  
  // it automatically throws an exception if the cast does not work
  const feature::CCTAG_Regions &queryRegions = genQueryRegions.getRegions<feature::CCTAG_Regions>(_cctagDescType);
  
  // a map containing for each pair <pt3D_id, pt2D_id> the number of times that 
  // the association has been seen
  std::map<IndMatch3D2D, std::size_t > occurences;
  sfm::ImageLocalizerMatchData resectionData;
  

  std::vector<voctree::DocMatch> matchedImages;
  system::Timer timer;
  getAllAssociations(queryRegions, imageSize, *param, randomNumberGenerator, occurences, resectionData.pt2D, resectionData.pt3D, matchedImages, imagePath);
  
  resectionData.vec_descType.resize(resectionData.pt2D.cols(), _cctagDescType);
  ALICEVISION_LOG_DEBUG("[Matching]\tRetrieving associations took " << timer.elapsedMs() << "ms");
  
  const std::size_t numCollectedPts = occurences.size();
  
  // create an vector of <feat3D_id, feat2D_id>
  std::vector<IndMatch3D2D> associationIDs;
  associationIDs.reserve(numCollectedPts);

  for(const auto &ass : occurences)
  {
    // recopy the associations IDs in the vector
    associationIDs.push_back(ass.first);
  }
  
  assert(associationIDs.size() == numCollectedPts);
  assert(resectionData.pt2D.cols() == numCollectedPts);
  assert(resectionData.pt3D.cols() == numCollectedPts);

  geometry::Pose3 pose;
  
  timer.reset();
  // estimate the pose
  resectionData.error_max = param->_errorMax;
  ALICEVISION_LOG_DEBUG("[poseEstimation]\tEstimating camera pose...");
  const bool bResection = sfm::SfMLocalizer::Localize(imageSize,
                                                      // pass the input intrinsic if they are valid, null otherwise
                                                      (useInputIntrinsics) ? &queryIntrinsics : nullptr,
                                                      randomNumberGenerator,
                                                      resectionData,
                                                      pose,
                                                      param->_resectionEstimator);
  
  if(!bResection)
  {
    ALICEVISION_LOG_DEBUG("[poseEstimation]\tResection failed");
    if(!param->_visualDebug.empty() && !imagePath.empty())
    {
//      namespace bfs = boost::filesystem;
//      matching::saveFeatures2SVG(imagePath,
//                                 imageSize,
//                                 resectionData.pt2D,
//                                 param._visualDebug + "/" + bfs::path(imagePath).stem().string() + ".associations.svg");
    }
    localizationResult = LocalizationResult(resectionData, associationIDs, pose, queryIntrinsics, matchedImages, bResection);
    return localizationResult.isValid();
  }
  ALICEVISION_LOG_DEBUG("[poseEstimation]\tResection SUCCEDED");

  ALICEVISION_LOG_DEBUG("R est\n" << pose.rotation());
  ALICEVISION_LOG_DEBUG("t est\n" << pose.translation());

  // if we didn't use the provided intrinsics, estimate K from the projection
  // matrix estimated by the localizer and initialize the queryIntrinsics with
  // it and the image size. This will provide a first guess for the refine function
  if(!useInputIntrinsics)
  {
    // Decompose P matrix
    Mat3 K_, R_;
    Vec3 t_;
    // Decompose the projection matrix  to get K, R and t using 
    // RQ decomposition
    KRt_from_P(resectionData.projection_matrix, &K_, &R_, &t_);
    queryIntrinsics.setK(K_);
    ALICEVISION_LOG_DEBUG("K estimated\n" << K_);
    queryIntrinsics.setWidth(imageSize.first);
    queryIntrinsics.setHeight(imageSize.second);
  }

  // refine the estimated pose
  ALICEVISION_LOG_DEBUG("[poseEstimation]\tRefining estimated pose");
  const bool b_refine_pose = true;
  const bool refineStatus = sfm::SfMLocalizer::RefinePose(&queryIntrinsics,
                                                            pose,
                                                            resectionData,
                                                            b_refine_pose,
                                                            param->_refineIntrinsics);
  if(!refineStatus)
    ALICEVISION_LOG_DEBUG("Refine pose failed.");

  if(!param->_visualDebug.empty() && !imagePath.empty())
  {
    //@todo save image with cctag with different code color for inliers
  }
  
  ALICEVISION_LOG_DEBUG("[poseEstimation]\tPose estimation took " << timer.elapsedMs() << "ms.");

  localizationResult = LocalizationResult(resectionData, associationIDs, pose, queryIntrinsics, matchedImages, refineStatus);

  {
    // just debugging this block can be safely removed or commented out
    ALICEVISION_LOG_DEBUG("R refined\n" << pose.rotation());
    ALICEVISION_LOG_DEBUG("t refined\n" << pose.translation());
    ALICEVISION_LOG_DEBUG("K refined\n" << queryIntrinsics.K());

    const Mat2X residuals = localizationResult.computeInliersResiduals();

    const auto sqrErrors = (residuals.cwiseProduct(residuals)).colwise().sum();
    ALICEVISION_LOG_DEBUG("RMSE = " << std::sqrt(sqrErrors.mean())
                << " min = " << std::sqrt(sqrErrors.minCoeff())
                << " max = " << std::sqrt(sqrErrors.maxCoeff()));
  }

  return localizationResult.isValid();
  
  
}

CCTagLocalizer::~CCTagLocalizer()
{
}

// subposes is n-1 as we consider the first camera as the main camera and the 
// reference frame of the grid
bool CCTagLocalizer::localizeRig(const std::vector<image::Image<float>> & vec_imageGrey,
                                 const LocalizerParameters *parameters,
                                 std::mt19937 & randomNumberGenerator,
                                 std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                                 const std::vector<geometry::Pose3 > &vec_subPoses,
                                 geometry::Pose3 &rigPose,
                                 std::vector<localization::LocalizationResult> & vec_locResults)
{
  const CCTagLocalizer::Parameters *param = static_cast<const CCTagLocalizer::Parameters *>(parameters);
  if(!param)
  {
    throw std::invalid_argument("The CCTag localizer parameters are not in the right format.");
  }
  const size_t numCams = vec_imageGrey.size();
  assert(numCams == vec_queryIntrinsics.size());
  assert(numCams == vec_subPoses.size() + 1);

  std::vector<feature::MapRegionsPerDesc> vec_queryRegions(numCams);
  std::vector<std::pair<std::size_t, std::size_t> > vec_imageSize;
  
  //@todo parallelize?
  for(size_t i = 0; i < numCams; ++i)
  {
    image::Image<unsigned char> imageGrayUChar; // cctag image describer don't support float image
    imageGrayUChar = (vec_imageGrey.at(i).GetMat() * 255.f).cast<unsigned char>();

    // extract descriptors and features from each image
    ALICEVISION_LOG_DEBUG("[features]\tExtract CCTag from query image...");
    _imageDescriber.setConfigurationPreset(param->_featurePreset);
    _imageDescriber.describe(imageGrayUChar, vec_queryRegions[i][_imageDescriber.getDescriberType()]);
    ALICEVISION_LOG_DEBUG("[features]\tExtract CCTAG done: found " <<  vec_queryRegions[i].at(_imageDescriber.getDescriberType())->RegionCount() << " features");
    // add the image size for this image
    vec_imageSize.emplace_back(vec_imageGrey[i].Width(), vec_imageGrey[i].Height());
  }
  assert(vec_imageSize.size() == vec_queryRegions.size());
          
  return localizeRig(vec_queryRegions,
                     vec_imageSize,
                     parameters,
                     randomNumberGenerator,
                     vec_queryIntrinsics,
                     vec_subPoses,
                     rigPose,
                     vec_locResults);
}

bool CCTagLocalizer::localizeRig(const std::vector<feature::MapRegionsPerDesc> & vec_queryRegions,
                                 const std::vector<std::pair<std::size_t, std::size_t> > &vec_imageSize,
                                 const LocalizerParameters *parameters,
                                 std::mt19937 & randomNumberGenerator,
                                 std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                                 const std::vector<geometry::Pose3 > &vec_subPoses,
                                 geometry::Pose3 &rigPose,
                                 std::vector<LocalizationResult>& vec_locResults)
{
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENGV)
  if(!parameters->_useLocalizeRigNaive)
  {
    ALICEVISION_LOG_DEBUG("Using localizeRig_opengv()");
    return localizeRig_opengv(vec_queryRegions,
                              vec_imageSize,
                              parameters,
                              randomNumberGenerator,
                              vec_queryIntrinsics,
                              vec_subPoses,
                              rigPose,
                              vec_locResults);
  }
  else
#endif
  {
    if(!parameters->_useLocalizeRigNaive)
      ALICEVISION_LOG_DEBUG("OpenGV is not available. Fallback to localizeRig_naive().");
    return localizeRig_naive(vec_queryRegions,
                             vec_imageSize,
                             parameters,
                             randomNumberGenerator,
                             vec_queryIntrinsics,
                             vec_subPoses,
                             rigPose,
                             vec_locResults);
  }
}

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENGV)
bool CCTagLocalizer::localizeRig_opengv(const std::vector<feature::MapRegionsPerDesc> & vec_queryRegions,
                                 const std::vector<std::pair<std::size_t, std::size_t> > &imageSize,
                                 const LocalizerParameters *parameters,
                                 std::mt19937 & randomNumberGenerator,
                                 std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                                 const std::vector<geometry::Pose3 > &vec_subPoses,
                                 geometry::Pose3 &rigPose,
                                 std::vector<LocalizationResult>& vec_locResults)
{
  const size_t numCams = vec_queryRegions.size();
  assert(numCams == vec_queryIntrinsics.size());
  assert(numCams == vec_subPoses.size() + 1);
  
  vec_locResults.clear();
  vec_locResults.reserve(numCams);

  const CCTagLocalizer::Parameters *param = static_cast<const CCTagLocalizer::Parameters *>(parameters);
  if(!param)
  {
    throw std::invalid_argument("The CCTag localizer parameters are not in the right format.");
  }
  
  // each element of the vector is a map containing for each pair <pt3D_id, pt2D_id> 
  // the number of times that the association has been seen. One element fo the 
  // vector for each camera.
  std::vector<OccurenceMap> vec_occurrences(numCams);
  std::vector<Mat> vec_pts3D(numCams);
  std::vector<Mat> vec_pts2D(numCams);
  std::vector<std::vector<voctree::DocMatch> > vec_matchedImages(numCams);

  // for each camera retrieve the associations
  //@todo parallelize?
  size_t numAssociations = 0;
  for( size_t i = 0; i < numCams; ++i )
  {
    // this map is used to collect the 2d-3d associations as we go through the images
    // the key is a pair <Id3D, Id2d>
    // the element is the pair 3D point - 2D point
    auto &occurrences = vec_occurrences[i];
    auto &matchedImages = vec_matchedImages[i];
    Mat &pts3D = vec_pts3D[i];
    Mat &pts2D = vec_pts2D[i];
    const feature::CCTAG_Regions &queryRegions = vec_queryRegions[i].getRegions<feature::CCTAG_Regions>(_cctagDescType);
    getAllAssociations(queryRegions, imageSize[i],*param, randomNumberGenerator, occurrences, pts2D, pts3D, matchedImages);
    numAssociations += occurrences.size();
  }
  
  // @todo Here it could be possible to filter the associations according to their
  // occurrences, eg giving priority to those associations that are more frequent

  const size_t minNumAssociations = 4;  //possible parameter?
  if(numAssociations < minNumAssociations)
  {
    ALICEVISION_LOG_DEBUG("[poseEstimation]\tonly " << numAssociations << " have been found, not enough to do the resection!");
    for(std::size_t cam = 0; cam < numCams; ++cam)
    {
      // empty result with isValid set to false
      vec_locResults.emplace_back();
    }
    return false;
  }

  std::vector<std::vector<std::size_t> > vec_inliers;
  const EstimationStatus status = rigResection(vec_pts2D,
                                        vec_pts3D,
                                        vec_queryIntrinsics,
                                        vec_subPoses,
                                        nullptr,
                                        rigPose,
                                        vec_inliers,
                                        param->_angularThreshold);

  if(!status.isValid)
  {
    for(std::size_t cam = 0; cam < numCams; ++cam)
    {
      // empty result with isValid set to false
      vec_locResults.emplace_back();
    }
    return false;
  }
  
  {
    if(vec_inliers.size() < numCams)
    {
      // in general the inlier should be spread among different cameras
      ALICEVISION_CERR("WARNING: RIG Voctree Localizer: Inliers in " 
              << vec_inliers.size() << " cameras on a RIG of " 
              << numCams << " cameras.");
    }

    for(std::size_t camID = 0; camID < vec_inliers.size(); ++camID)
      ALICEVISION_LOG_DEBUG("#inliers for cam " << camID << ": " << vec_inliers[camID].size());
    
    ALICEVISION_LOG_DEBUG("Pose after resection:");
    ALICEVISION_LOG_DEBUG("Rotation: " << rigPose.rotation());
    ALICEVISION_LOG_DEBUG("Centre: " << rigPose.center());
    
    // debugging stats
    // print the reprojection error for inliers (just debugging purposes)
    printRigRMSEStats(vec_pts2D, vec_pts3D, vec_queryIntrinsics, vec_subPoses, rigPose, vec_inliers);
  }
  
//  const bool refineOk = refineRigPose(vec_pts2D,
//                                      vec_pts3D,
//                                      vec_inliers,
//                                      vec_queryIntrinsics,
//                                      vec_subPoses,
//                                      rigPose);
  aliceVision::system::Timer timer;
  const std::size_t minNumPoints = 4;
  const bool refineOk = iterativeRefineRigPose(vec_pts2D,
                                               vec_pts3D,
                                               vec_queryIntrinsics,
                                               vec_subPoses,
                                               param->_errorMax,
                                               minNumPoints,
                                               vec_inliers,
                                               rigPose);
  ALICEVISION_LOG_DEBUG("Iterative refinement took " << timer.elapsedMs() << "ms");
  
  {
    // debugging stats
    // print the reprojection error for inliers (just debugging purposes)
    printRigRMSEStats(vec_pts2D, vec_pts3D, vec_queryIntrinsics, vec_subPoses, rigPose, vec_inliers);
  }
  
  // create localization results
  for(std::size_t cam = 0; cam < numCams; ++cam)
  {

    const auto &intrinsics = vec_queryIntrinsics[cam];

    // compute the (absolute) pose of each camera: for the main camera it's the 
    // rig pose, for the others, combine the subpose with the rig pose
    geometry::Pose3 pose;
    if(cam == 0)
    {
      pose = rigPose;
    }
    else
    {
      // main camera: q1 ~ [R1 t1] Q = [I 0] A   where A = [R1 t1] Q  
      // another camera: q2 ~ [R2 t2] Q = [R2 t2]*inv([R1 t1]) A 
      // and subPose12 = [R12 t12] = [R2 t2]*inv([R1 t1])
      // With rigResection() we compute [R1 t1] (aka rigPose), hence:
      // subPose12 = [R12 t12] = [R2 t2]*inv([R1 t1]) and we need [R2 t2], ie the absolute pose
      // => [R1 t1] * subPose12 = [R2 t2]
      // => rigPose * subPose12 = [R2 t2]
      pose = vec_subPoses[cam-1] * rigPose;
    }
    
    // create matchData
    sfm::ImageLocalizerMatchData matchData;
    matchData.vec_inliers = vec_inliers[cam];
    matchData.error_max = param->_errorMax;
    matchData.projection_matrix = intrinsics.getProjectiveEquivalent(pose);
    matchData.pt2D = vec_pts2D[cam];
    matchData.pt3D = vec_pts3D[cam];
    
    // create indMatch3D2D
    std::vector<IndMatch3D2D> indMatch3D2D;
    indMatch3D2D.reserve(matchData.pt2D.cols());
    const auto &occurrences = vec_occurrences[cam];
    for(const auto &ass : occurrences)
    {
      // recopy the associations IDs in the vector
      indMatch3D2D.push_back(ass.first);
    }
    
    vec_locResults.emplace_back(matchData, indMatch3D2D, pose, intrinsics, vec_matchedImages[cam], refineOk);
  }
  
  if(!refineOk)
  {
    ALICEVISION_LOG_DEBUG("[poseEstimation]\tRefine failed.");
    return false;
  }
  
  return true;
  
}
  
#endif //ALICEVISION_HAVE_OPENGV

// subposes is n-1 as we consider the first camera as the main camera and the 
// reference frame of the grid
bool CCTagLocalizer::localizeRig_naive(const std::vector<feature::MapRegionsPerDesc> & vec_queryRegions,
                                 const std::vector<std::pair<std::size_t, std::size_t> > &imageSize,
                                 const LocalizerParameters *parameters,
                                 std::mt19937 & randomNumberGenerator,
                                 std::vector<camera::PinholeRadialK3 > &vec_queryIntrinsics,
                                 const std::vector<geometry::Pose3 > &vec_subPoses,
                                 geometry::Pose3 &rigPose,
                                 std::vector<LocalizationResult>& vec_localizationResults)
{
  const CCTagLocalizer::Parameters *param = dynamic_cast<const CCTagLocalizer::Parameters *>(parameters);
  if(!param)
  {
    throw std::invalid_argument("The CCTag localizer parameters are not in the right format.");
  }

  const size_t numCams = vec_queryRegions.size();
  
  assert(numCams==vec_queryIntrinsics.size());
  assert(numCams==vec_subPoses.size()+1);
  assert(numCams==imageSize.size());

  vec_localizationResults.resize(numCams);
    
  // this is basic, just localize each camera alone
  //@todo parallelize?
  std::vector<bool> isLocalized(numCams, false);
  for(size_t i = 0; i < numCams; ++i)
  {
    isLocalized[i] = localize(vec_queryRegions[i], imageSize[i], param, randomNumberGenerator, true /*useInputIntrinsics*/, vec_queryIntrinsics[i], vec_localizationResults[i]);
    if(!isLocalized[i])
    {
      ALICEVISION_CERR("Could not localize camera " << i);
      // even if it is not localize we can try to go on and do with the cameras we have
    }
  }
  
  // ** 'easy' cases in which we don't need further processing **
  
  const std::size_t numLocalizedCam = std::count(isLocalized.begin(), isLocalized.end(), true);
  
  // no camera has be localized
  if(numLocalizedCam == 0)
  {
    ALICEVISION_LOG_DEBUG("No camera has been localized!!!");
    return false;
  }
  
  ALICEVISION_LOG_DEBUG("Localized cameras: " << numLocalizedCam << "/" << numCams);
  
  // if there is only one camera (the main one)
  if(numCams==1)
  {
    // there is only the main camera, not much else to do, the position is already
    // refined by the call to localize
    //set the pose
    rigPose = vec_localizationResults[0].getPose();
  }
  else
  {

    // find the index of the first localized camera
    const std::size_t idx = std::distance(isLocalized.begin(), 
                                          std::find(isLocalized.begin(), isLocalized.end(), true));
    
    // useless safeguard as there should be at least 1 element at this point but
    // better safe than sorry
    assert(idx < isLocalized.size());
    
    ALICEVISION_LOG_DEBUG("Index of the first localized camera: " << idx);
    
    // if the only localized camera is the main camera
    if(idx==0)
    {
      // just give its pose
      rigPose = vec_localizationResults[0].getPose();
    }
    else
    {
      // main camera: q1 ~ [R1 t1] Q = [I 0] A   where A = [R1 t1] Q  
      // another camera: q2 ~ [R2 t2] Q = [R2 t2]*inv([R1 t1]) A   and subPose12 = [R12 t12] = [R2 t2]*inv([R1 t1])
      // with the localization localize() we have computed [R2 t2], hence:
      // q2 ~ [R2 t2] Q = [R12 t12]*inv([R12 t12]) * [R2 t2] Q
      // and inv([R12 t12]) * [R2 t2] is the pose of the main camera
      
      // recover the rig pose using the subposes
      rigPose = vec_subPoses[idx-1].inverse() * vec_localizationResults[idx].getPose();
    }
  }
  
  // ** otherwise run a BA with the localized cameras
  const bool refineOk = refineRigPose(vec_subPoses, vec_localizationResults, rigPose);
  
  if(!refineOk)
  {
    ALICEVISION_LOG_DEBUG("[poseEstimation]\tRig pose refinement failed.");
    return false;
  }
  
  updateRigPoses(vec_localizationResults, rigPose, vec_subPoses);
  
  return true;
}


void CCTagLocalizer::getAllAssociations(const feature::CCTAG_Regions &queryRegions,
                                        const std::pair<std::size_t, std::size_t> &imageSize,
                                        const CCTagLocalizer::Parameters &param,
                                        std::mt19937 & randomNumberGenerator,
                                        OccurenceMap & out_occurences,
                                        Mat &out_pt2D,
                                        Mat &out_pt3D,
                                        std::vector<voctree::DocMatch>& out_matchedImages,
                                        const std::string& imagePath) const
{
  std::vector<IndexT> nearestKeyFrames;
  nearestKeyFrames.reserve(param._nNearestKeyFrames);
  
  kNearestKeyFrames(queryRegions,
                    _cctagDescType,
                    _regionsPerView,
                    param._nNearestKeyFrames,
                    nearestKeyFrames);
  
  out_matchedImages.clear();
  out_matchedImages.reserve(nearestKeyFrames.size());
  
  ALICEVISION_LOG_DEBUG("nearestKeyFrames.size() = " << nearestKeyFrames.size());
  for(const IndexT keyframeId : nearestKeyFrames)
  {
    ALICEVISION_LOG_DEBUG(keyframeId);
    ALICEVISION_LOG_DEBUG(_sfm_data.getViews().at(keyframeId)->getImagePath());
    const feature::Regions& matchedRegions = _regionsPerView.getRegions(keyframeId, _cctagDescType);
    const ReconstructedRegionsMapping& regionsMapping = _reconstructedRegionsMappingPerView.at(keyframeId).at(_cctagDescType);
    const feature::CCTAG_Regions & matchedCCtagRegions = dynamic_cast<const feature::CCTAG_Regions &>(matchedRegions);

    // Matching
    std::vector<matching::IndMatch> vec_featureMatches;
    viewMatching(queryRegions, matchedCCtagRegions, vec_featureMatches);
    ALICEVISION_LOG_DEBUG("[matching]\tFound "<< vec_featureMatches.size() <<" matches.");
    
    out_matchedImages.emplace_back(keyframeId, vec_featureMatches.size());
    
    if(!param._visualDebug.empty() && !imagePath.empty())
    {
      namespace bfs = boost::filesystem;
      const sfmData::View *mview = _sfm_data.getViews().at(keyframeId).get();
      const std::string queryImage = bfs::path(imagePath).stem().string();
      const std::string matchedImage = bfs::path(mview->getImagePath()).stem().string();
      const std::string matchedPath = mview->getImagePath();

      // the directory where to save the feature matches
      const auto baseDir = bfs::path(param._visualDebug) / queryImage;
      if((!bfs::exists(baseDir)))
      {
        ALICEVISION_LOG_DEBUG("created " << baseDir.string());
        bfs::create_directories(baseDir);
      }
      
      // the final filename for the output svg file as a composition of the query
      // image and the matched image
      auto outputName = baseDir / queryImage;
      outputName += "_";
      outputName += matchedImage;
      outputName += ".svg";
      
      const bool showNotMatched = true;
      matching::saveCCTagMatches2SVG(imagePath, 
                                     imageSize, 
                                     queryRegions,
                                     matchedPath,
                                     std::make_pair(mview->getWidth(), mview->getHeight()),
                                     matchedCCtagRegions,
                                     vec_featureMatches,
                                     outputName.string(),
                                     showNotMatched ); 
    }
    
    // Recover the 2D-3D associations from the matches 
    // Each matched feature in the current similar image is associated to a 3D point,
    // hence we can recover the 2D-3D associations to estimate the pose
    
    // Get the 3D points associated to each matched feature
    for(const matching::IndMatch& featureMatch : vec_featureMatches)
    {
      // the ID of the 3D point
      const IndexT pt3D_id = regionsMapping._associated3dPoint[featureMatch._j];
      const IndexT pt2D_id = featureMatch._i;
      
      const IndMatch3D2D key(pt3D_id, _cctagDescType, pt2D_id);
      if(out_occurences.count(key))
      {
        out_occurences[key]++;
      }
      else
      {
        out_occurences[key] = 1;
      }
    }
  }
      
  const size_t numCollectedPts = out_occurences.size();
  ALICEVISION_LOG_DEBUG("[matching]\tCollected "<< numCollectedPts <<" associations.");
  
  {
    // just debugging statistics, this block can be safely removed    
    std::size_t maxOcc = 0;
    for(const auto &idx : out_occurences)
    {
      const auto &key = idx.first;
      const auto &value = idx.second;
      ALICEVISION_LOG_DEBUG("[matching]\tAssociations "
              << feature::EImageDescriberType_enumToString(key.descType) << " "
              << key.landmarkId << "," << key.featId <<" found "
              << value << " times.");
      if(value > maxOcc)
        maxOcc = value;
    }
    
    std::size_t numOccTreated = 0;
    for(std::size_t value = 1; value < maxOcc; ++value)
    {
      std::size_t counter = 0;
      for(const auto &idx : out_occurences)
      {
        if(idx.second == value)
        {
          ++counter;
        }
      }
      if(counter>0)
        ALICEVISION_LOG_DEBUG("[matching]\tThere are " << counter
                    << " associations occurred " << value << " times ("
                    << 100.0 * counter / (double) numCollectedPts << "%)");
      numOccTreated += counter;
      if(numOccTreated >= numCollectedPts)
        break;
    }
  }

  out_pt2D = Mat2X(2, numCollectedPts);
  out_pt3D = Mat3X(3, numCollectedPts);

  size_t index = 0;
  for(const auto &idx : out_occurences)
  {
    // recopy all the points in the matching structure
    const IndexT pt3D_id = idx.first.landmarkId;
    const IndexT pt2D_id = idx.first.featId;
      
    out_pt2D.col(index) = queryRegions.GetRegionPosition(pt2D_id);
    out_pt3D.col(index) = _sfm_data.getLandmarks().at(pt3D_id).X;
    ++index;
  }
}


void kNearestKeyFrames(const feature::CCTAG_Regions & queryRegions,
                       feature::EImageDescriberType cctagDescType,
                       const feature::RegionsPerView & regionsPerView,
                       std::size_t nNearestKeyFrames,
                       std::vector<IndexT> & out_kNearestFrames,
                       float similarityThreshold /*=.0f*/)
{
  out_kNearestFrames.clear();
  
  // A std::multimap is used instead of a std::map because is very likely that the
  // similarity measure is equal for a subset of views in the CCTag regions case.
  std::multimap<float, IndexT> sortedViewSimilarities;
  
  for(const auto & keyFrame : regionsPerView.getData())
  {
    const feature::CCTAG_Regions& keyFrameCCTagRegions = keyFrame.second.getRegions<feature::CCTAG_Regions>(cctagDescType);
    const float similarity = viewSimilarity(queryRegions, keyFrameCCTagRegions);
    sortedViewSimilarities.emplace(similarity, keyFrame.first);
  }
  
  std::size_t counter = 0;
  out_kNearestFrames.reserve(nNearestKeyFrames);
  for (auto rit = sortedViewSimilarities.crbegin(); rit != sortedViewSimilarities.crend(); ++rit)
  {
    if(rit->first < similarityThreshold)
      // since it is ordered, the first having smaller similarity guarantees that
      // there won't be other useful kframes
      break;
    
    out_kNearestFrames.push_back(rit->second);
    ++counter;
    
    if (counter == nNearestKeyFrames)
      break;
  }
}
 
void viewMatching(const feature::CCTAG_Regions & regionsA,
                  const feature::CCTAG_Regions & regionsB,
                  std::vector<matching::IndMatch> & out_featureMatches)
{
  out_featureMatches.clear();
  
  for(std::size_t i=0 ; i < regionsA.Descriptors().size() ; ++i)
  {
    const IndexT cctagIdA = feature::getCCTagId(regionsA.Descriptors()[i]);
    // todo: Should be change to: Find in regionsB.Descriptors() the nearest 
    // descriptor to descriptorA. Currently, a cctag descriptor encode directly
    // the cctag id, then the id equality is tested.
    for(std::size_t j=0 ; j < regionsB.Descriptors().size() ; ++j)
    {
      const IndexT cctagIdB = feature::getCCTagId(regionsB.Descriptors()[j]);
      if ( cctagIdA == cctagIdB )
      {
        out_featureMatches.emplace_back(i,j);
        break;
      }
    }
  }
}
 
 
 
float viewSimilarity(const feature::CCTAG_Regions & regionsA,
                     const feature::CCTAG_Regions & regionsB)
{
  assert(regionsA.DescriptorLength() == regionsB.DescriptorLength()); 
  
  const std::bitset<128> descriptorViewA = constructCCTagViewDescriptor(regionsA.Descriptors());
  const std::bitset<128> descriptorViewB = constructCCTagViewDescriptor(regionsB.Descriptors());
  
  // The similarity is the sum of all the cctags sharing the same id visible in both views.
  return (descriptorViewA & descriptorViewB).count();
}

std::bitset<128> constructCCTagViewDescriptor(const std::vector<feature::CCTAG_Regions::DescriptorT> & vCCTagDescriptors)
{
  std::bitset<128> descriptorView;
  for(const auto & cctagDescriptor : vCCTagDescriptors )
  {
    const IndexT cctagId = feature::getCCTagId(cctagDescriptor);
    if ( cctagId != UndefinedIndexT)
    {
      descriptorView.set(cctagId, true);
    }
  }
  return descriptorView;
}

} // localization
} // aliceVision

