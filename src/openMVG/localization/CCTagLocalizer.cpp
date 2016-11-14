#include "CCTagLocalizer.hpp"
#include "reconstructed_regions.hpp"
#include "optimization.hpp"
#include "rigResection.hpp"

#include <openMVG/features/svgVisualization.hpp>
#include <openMVG/sfm/sfm_data_io.hpp>
#include <openMVG/matching/indMatch.hpp>
#include <openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp>
#include <openMVG/system/timer.hpp>
#include <openMVG/logger.hpp>

#include <cctag/ICCTag.hpp>

#include <boost/filesystem.hpp>

#include <algorithm>
#include <sstream>

namespace openMVG {
namespace localization {

CCTagLocalizer::CCTagLocalizer(const std::string &sfmFilePath,
                               const std::string &descriptorsFolder)
    : _cudaPipe( 0 )
{
  using namespace openMVG::features;

  // load the sfm data containing the 3D reconstruction info
  if (!Load(_sfm_data, sfmFilePath, sfm::ESfM_Data::ALL)) 
  {
    OPENMVG_CERR("The input SfM_Data file "<< sfmFilePath << " cannot be read.");
    OPENMVG_CERR("\n\nIf the error says \"JSON Parsing failed - provided NVP not found\" "
        "it's likely that you have to convert your sfm_data to a recent version supporting "
        "polymorphic Views. You can run the python script convertSfmData.py to update an existing sfmdata.");
    throw std::invalid_argument("The input SfM_Data file "+ sfmFilePath + " cannot be read.");
  }

  // this block is used to get the type of features (by default SIFT) used
  // for the reconstruction
  const std::string sImage_describer = stlplus::create_filespec(descriptorsFolder, "image_describer", "json");
  std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
  if(!regions_type)
  {
    OPENMVG_CERR("Invalid: "
            << sImage_describer << " regions type file.");
    throw std::invalid_argument("Invalid: "+ sImage_describer + " regions type file.");
  }
  
  bool loadSuccessful = loadReconstructionDescriptors(_sfm_data, descriptorsFolder);
  
  if(!loadSuccessful)
  {
    OPENMVG_CERR("Unable to load the descriptors");
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


bool CCTagLocalizer::loadReconstructionDescriptors(const sfm::SfM_Data & sfm_data,
                                                   const std::string & feat_directory)
{
  C_Progress_display my_progress_bar(sfm_data.GetViews().size(),
                                     std::cout, "\n- Regions Loading -\n");

  OPENMVG_LOG_DEBUG("Build observations per view");
  // Build observations per view
  std::map<IndexT, std::vector<FeatureInImage> > observationsPerView;
  for(auto landmarkValue : sfm_data.structure)
  {
    IndexT trackId = landmarkValue.first;
    sfm::Landmark& landmark = landmarkValue.second;
    for(auto obs : landmark.obs)
    {
      const IndexT viewId = obs.first;
      const sfm::Observation& obs2d = obs.second;
      observationsPerView[viewId].push_back(FeatureInImage(obs2d.id_feat, trackId));
    }
  }
  for(auto featuresInImage : observationsPerView)
  {
    std::sort(featuresInImage.second.begin(), featuresInImage.second.end());
  }
  
  OPENMVG_LOG_DEBUG("Load Features and Descriptors per view");
  std::vector<bool> presentIds(128,false); // @todo Assume a maximum library size of 128 unique ids.
  std::vector<int> counterCCtagsInImage = {0, 0, 0, 0, 0, 0};
  // Read for each view the corresponding regions and store them
  for(const auto &iter : sfm_data.GetViews())
  {
    const IndexT id_view = iter.second->id_view;
    Reconstructed_RegionsCCTag& reconstructedRegion = _regions_per_view[id_view];

    const std::string &sImageName = iter.second.get()->s_Img_path;
    std::string featFilepath = stlplus::create_filespec(feat_directory, std::to_string(iter.first), ".feat");
    std::string descFilepath = stlplus::create_filespec(feat_directory, std::to_string(iter.first), ".desc");

    if(!(stlplus::is_file(featFilepath) && stlplus::is_file(descFilepath)))
    {
      // legacy compatibility, if the features are not named using the UID convention
      // let's try with the old-fashion naming convention
      const std::string basename = stlplus::basename_part(sImageName);
      featFilepath = stlplus::create_filespec(feat_directory, basename, ".feat");
      descFilepath = stlplus::create_filespec(feat_directory, basename, ".desc");
      if(!(stlplus::is_file(featFilepath) && stlplus::is_file(descFilepath)))
      {
        OPENMVG_CERR("Cannot find the features for image " << sImageName 
                << " neither using the UID naming convention nor the image name based convention");
        return false;
      }
    }

    if(!reconstructedRegion._regions.Load(featFilepath, descFilepath))
    {
      OPENMVG_CERR("Invalid regions files for the view: " << sImageName);
      return false;
    }
    
    // Filter descriptors to keep only the 3D reconstructed points
    reconstructedRegion.filterCCTagRegions(observationsPerView[id_view]);
    
    // Update the visibility mask
    reconstructedRegion.updateLandmarksVisibility(presentIds);
    
    ++my_progress_bar;
  }

  {
    // just debugging stuff -- print for each image the visible reconstructed cctag
    // and create an histogram of cctags per image
    for(const auto &iter : sfm_data.GetViews())
    {
      const IndexT id_view = iter.second->id_view;
      Reconstructed_RegionsCCTag& reconstructedRegion = _regions_per_view[id_view];
      const std::string &sImageName = iter.second.get()->s_Img_path;
      std::stringstream ss;

      ss << "Image " << sImageName;
      if(reconstructedRegion._regions.Descriptors().size() == 0 )
      {
        counterCCtagsInImage[0] +=1;
        ss << " does not contain any cctag!!!";
      }
      else
      {
        ss << " contains CCTag Id: ";
        for(const auto &desc : reconstructedRegion._regions.Descriptors())
        {
          const IndexT cctagIdA = features::getCCTagId(desc);
          if(cctagIdA != UndefinedIndexT)
            ss << cctagIdA << " ";
        }
        // Update histogram
        int countcctag = reconstructedRegion._regions.Descriptors().size();
        if(countcctag >= 5)
          counterCCtagsInImage[5] +=1;
        else
          counterCCtagsInImage[countcctag] += 1;
      }
      OPENMVG_LOG_DEBUG(ss.str());
    }
    
    // Display histogram
    OPENMVG_LOG_DEBUG("Histogram of number of cctags in images :");
    for(std::size_t i = 0; i < 5; i++)
      OPENMVG_LOG_DEBUG("Images with " << i << "  CCTags : " << counterCCtagsInImage[i]);
    OPENMVG_LOG_DEBUG("Images with 5+ CCTags : " << counterCCtagsInImage[5]);

    // Display the cctag ids over all cctag landmarks present in the database
    OPENMVG_LOG_DEBUG("Found " << std::count(presentIds.begin(), presentIds.end(), true) 
            << " CCTag in the database with " << _sfm_data.GetLandmarks().size() << " associated 3D points\n"
            "The CCTag id in the database are: ");
    for(std::size_t i = 0; i < presentIds.size(); ++i)
    { 
      if(presentIds[i])
        OPENMVG_LOG_DEBUG(i + 1);
    }

  }
  
  return true;
}

bool CCTagLocalizer::localize(const image::Image<unsigned char> & imageGrey,
                              const LocalizerParameters *parameters,
                              bool useInputIntrinsics,
                              cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
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
  OPENMVG_LOG_DEBUG("[features]\tExtract CCTag from query image");
  std::unique_ptr<features::Regions> tmpQueryRegions(new features::CCTAG_Regions());

  _image_describer.setCudaPipe( _cudaPipe );
  _image_describer.Set_configuration_preset(param->_featurePreset);
  _image_describer.Describe(imageGrey, tmpQueryRegions);
  OPENMVG_LOG_DEBUG("[features]\tExtract CCTAG done: found " << tmpQueryRegions->RegionCount() << " features");
  
  std::pair<std::size_t, std::size_t> imageSize = std::make_pair(imageGrey.Width(),imageGrey.Height());
  
  if(!param->_visualDebug.empty() && !imagePath.empty())
  {
    // it automatically throws an exception if the cast does not work
    features::CCTAG_Regions &queryRegions = *dynamic_cast<features::CCTAG_Regions*> (tmpQueryRegions.get());
    
    // just debugging -- save the svg image with detected cctag
    features::saveCCTag2SVG(imagePath, 
                            imageSize, 
                            queryRegions, 
                            param->_visualDebug+"/"+bfs::path(imagePath).stem().string()+".svg");
  }
  return localize(tmpQueryRegions,
                  imageSize,
                  parameters,
                  useInputIntrinsics,
                  queryIntrinsics,
                  localizationResult,
                  imagePath);
}

void CCTagLocalizer::setCudaPipe( int i )
{
    _cudaPipe = i;
}

bool CCTagLocalizer::localize(const std::unique_ptr<features::Regions> &genQueryRegions,
                              const std::pair<std::size_t, std::size_t> &imageSize,
                              const LocalizerParameters *parameters,
                              bool useInputIntrinsics,
                              cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                              LocalizationResult & localizationResult,
                              const std::string& imagePath)
{
  namespace bfs = boost::filesystem;
  
  const CCTagLocalizer::Parameters *param = static_cast<const CCTagLocalizer::Parameters *>(parameters);
  if(!param)
  {
    throw std::invalid_argument("The CCTag localizer parameters are not in the right format.");
  }
  
  // it automatically throws an exception if the cast does not work
  features::CCTAG_Regions &queryRegions = *dynamic_cast<features::CCTAG_Regions*> (genQueryRegions.get());
  
  // a map containing for each pair <pt3D_id, pt2D_id> the number of times that 
  // the association has been seen
  std::map< std::pair<IndexT, IndexT>, std::size_t > occurences;
  sfm::Image_Localizer_Match_Data resectionData;
  

  std::vector<voctree::DocMatch> matchedImages;
  system::Timer timer;
  getAllAssociations(queryRegions, imageSize, *param, occurences, resectionData.pt2D, resectionData.pt3D, matchedImages, imagePath);
  OPENMVG_LOG_DEBUG("[Matching]\tRetrieving associations took " << timer.elapsedMs() << "ms");
  
  const std::size_t numCollectedPts = occurences.size();
  
  // create an vector of <feat3D_id, feat2D_id>
  std::vector<pair<IndexT, IndexT> > associationIDs;
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
  OPENMVG_LOG_DEBUG("[poseEstimation]\tEstimating camera pose...");
  const bool bResection = sfm::SfM_Localizer::Localize(imageSize,
                                                      // pass the input intrinsic if they are valid, null otherwise
                                                      (useInputIntrinsics) ? &queryIntrinsics : nullptr,
                                                      resectionData,
                                                      pose,
                                                      param->_resectionEstimator);
  
  if(!bResection)
  {
    OPENMVG_LOG_DEBUG("[poseEstimation]\tResection failed");
    if(!param->_visualDebug.empty() && !imagePath.empty())
    {
//      namespace bfs = boost::filesystem;
//      features::saveFeatures2SVG(imagePath,
//                                 imageSize,
//                                 resectionData.pt2D,
//                                 param._visualDebug + "/" + bfs::path(imagePath).stem().string() + ".associations.svg");
    }
    localizationResult = LocalizationResult(resectionData, associationIDs, pose, queryIntrinsics, matchedImages, bResection);
    return localizationResult.isValid();
  }
  OPENMVG_LOG_DEBUG("[poseEstimation]\tResection SUCCEDED");

  OPENMVG_LOG_DEBUG("R est\n" << pose.rotation());
  OPENMVG_LOG_DEBUG("t est\n" << pose.translation());

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
    KRt_From_P(resectionData.projection_matrix, &K_, &R_, &t_);
    queryIntrinsics.setK(K_);
    OPENMVG_LOG_DEBUG("K estimated\n" << K_);
    queryIntrinsics.setWidth(imageSize.first);
    queryIntrinsics.setHeight(imageSize.second);
  }

  // refine the estimated pose
  OPENMVG_LOG_DEBUG("[poseEstimation]\tRefining estimated pose");
  const bool b_refine_pose = true;
  const bool refineStatus = sfm::SfM_Localizer::RefinePose(&queryIntrinsics,
                                                            pose,
                                                            resectionData,
                                                            b_refine_pose,
                                                            param->_refineIntrinsics);
  if(!refineStatus)
    OPENMVG_LOG_DEBUG("Refine pose failed.");

  if(!param->_visualDebug.empty() && !imagePath.empty())
  {
    //@todo save image with cctag with different code color for inliers
  }
  
  OPENMVG_LOG_DEBUG("[poseEstimation]\tPose estimation took " << timer.elapsedMs() << "ms.");

  localizationResult = LocalizationResult(resectionData, associationIDs, pose, queryIntrinsics, matchedImages, refineStatus);

  {
    // just debugging this block can be safely removed or commented out
    OPENMVG_LOG_DEBUG("R refined\n" << pose.rotation());
    OPENMVG_LOG_DEBUG("t refined\n" << pose.translation());
    OPENMVG_LOG_DEBUG("K refined\n" << queryIntrinsics.K());

    const Mat2X residuals = localizationResult.computeInliersResiduals();

    const auto sqrErrors = (residuals.cwiseProduct(residuals)).colwise().sum();
    OPENMVG_LOG_DEBUG("RMSE = " << std::sqrt(sqrErrors.mean())
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
bool CCTagLocalizer::localizeRig(const std::vector<image::Image<unsigned char> > & vec_imageGrey,
                                 const LocalizerParameters *parameters,
                                 std::vector<cameras::Pinhole_Intrinsic_Radial_K3 > &vec_queryIntrinsics,
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

  std::vector<std::unique_ptr<features::Regions> > vec_queryRegions(numCams);
  std::vector<std::pair<std::size_t, std::size_t> > vec_imageSize;
  
  //@todo parallelize?
  for(size_t i = 0; i < numCams; ++i)
  {
    // extract descriptors and features from each image
    vec_queryRegions[i] = std::unique_ptr<features::Regions>(new features::CCTAG_Regions());
    OPENMVG_LOG_DEBUG("[features]\tExtract CCTag from query image...");
    _image_describer.Set_configuration_preset(param->_featurePreset);
    _image_describer.Describe(vec_imageGrey[i], vec_queryRegions[i]);
    OPENMVG_LOG_DEBUG("[features]\tExtract CCTAG done: found " <<  vec_queryRegions[i]->RegionCount() << " features");
    // add the image size for this image
    vec_imageSize.emplace_back(vec_imageGrey[i].Width(), vec_imageGrey[i].Height());
  }
  assert(vec_imageSize.size() == vec_queryRegions.size());
          
  return localizeRig(vec_queryRegions,
                     vec_imageSize,
                     parameters,
                     vec_queryIntrinsics,
                     vec_subPoses,
                     rigPose,
                     vec_locResults);
}

bool CCTagLocalizer::localizeRig(const std::vector<std::unique_ptr<features::Regions> > & vec_queryRegions,
                                 const std::vector<std::pair<std::size_t, std::size_t> > &vec_imageSize,
                                 const LocalizerParameters *parameters,
                                 std::vector<cameras::Pinhole_Intrinsic_Radial_K3 > &vec_queryIntrinsics,
                                 const std::vector<geometry::Pose3 > &vec_subPoses,
                                 geometry::Pose3 &rigPose,
                                 std::vector<LocalizationResult>& vec_locResults)
{
#ifdef HAVE_OPENGV
  if(!parameters->_useLocalizeRigNaive)
  {
    OPENMVG_LOG_DEBUG("Using localizeRig_naive()");
    return localizeRig_opengv(vec_queryRegions,
                              vec_imageSize,
                              parameters,
                              vec_queryIntrinsics,
                              vec_subPoses,
                              rigPose,
                              vec_locResults);
  }
  else
#endif
  {
    if(!parameters->_useLocalizeRigNaive)
      OPENMVG_LOG_DEBUG("OpenGV is not available. Fallback to localizeRig_naive().");
    return localizeRig_naive(vec_queryRegions,
                             vec_imageSize,
                             parameters,
                             vec_queryIntrinsics,
                             vec_subPoses,
                             rigPose,
                             vec_locResults);
  }
}

#ifdef HAVE_OPENGV
bool CCTagLocalizer::localizeRig_opengv(const std::vector<std::unique_ptr<features::Regions> > & vec_queryRegions,
                                 const std::vector<std::pair<std::size_t, std::size_t> > &imageSize,
                                 const LocalizerParameters *parameters,
                                 std::vector<cameras::Pinhole_Intrinsic_Radial_K3 > &vec_queryIntrinsics,
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
  std::vector<std::map< pair<IndexT, IndexT>, std::size_t > > vec_occurrences(numCams);
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
    features::CCTAG_Regions &queryRegions = *dynamic_cast<features::CCTAG_Regions*> (vec_queryRegions[i].get());
    getAllAssociations(queryRegions, imageSize[i],*param, occurrences, pts2D, pts3D, matchedImages);
    numAssociations += occurrences.size();
  }
  
  // @todo Here it could be possible to filter the associations according to their
  // occurrences, eg giving priority to those associations that are more frequent

  const size_t minNumAssociations = 4;  //possible parameter?
  if(numAssociations < minNumAssociations)
  {
    OPENMVG_LOG_DEBUG("[poseEstimation]\tonly " << numAssociations << " have been found, not enough to do the resection!");
    for(std::size_t cam = 0; cam < numCams; ++cam)
    {
      // empty result with isValid set to false
      vec_locResults.emplace_back();
    }
    return false;
  }

  std::vector<std::vector<std::size_t> > vec_inliers;
  const bool resectionOk = rigResection(vec_pts2D,
                                        vec_pts3D,
                                        vec_queryIntrinsics,
                                        vec_subPoses,
                                        rigPose,
                                        vec_inliers,
                                        param->_angularThreshold);

  if(!resectionOk)
  {
    for(std::size_t cam = 0; cam < numCams; ++cam)
    {
      // empty result with isValid set to false
      vec_locResults.emplace_back();
    }
    return resectionOk;
  }
  
  {
    if(vec_inliers.size() < numCams)
    {
      // in general the inlier should be spread among different cameras
      OPENMVG_CERR("WARNING: RIG Voctree Localizer: Inliers in " 
              << vec_inliers.size() << " cameras on a RIG of " 
              << numCams << " cameras.");
    }

    for(std::size_t camID = 0; camID < vec_inliers.size(); ++camID)
      OPENMVG_LOG_DEBUG("#inliers for cam " << camID << ": " << vec_inliers[camID].size());
    
    OPENMVG_LOG_DEBUG("Pose after resection:");
    OPENMVG_LOG_DEBUG("Rotation: " << rigPose.rotation());
    OPENMVG_LOG_DEBUG("Centre: " << rigPose.center());
    
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
  openMVG::system::Timer timer;
  const std::size_t minNumPoints = 4;
  const bool refineOk = iterativeRefineRigPose(vec_pts2D,
                                               vec_pts3D,
                                               vec_queryIntrinsics,
                                               vec_subPoses,
                                               param->_errorMax,
                                               minNumPoints,
                                               vec_inliers,
                                               rigPose);
  OPENMVG_LOG_DEBUG("Iterative refinement took " << timer.elapsedMs() << "ms");
  
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
    sfm::Image_Localizer_Match_Data matchData;
    matchData.vec_inliers = vec_inliers[cam];
    matchData.error_max = param->_errorMax;
    matchData.projection_matrix = intrinsics.get_projective_equivalent(pose);
    matchData.pt2D = vec_pts2D[cam];
    matchData.pt3D = vec_pts3D[cam];
    
    // create indMatch3D2D
    std::vector<pair<IndexT, IndexT> > indMatch3D2D;
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
    OPENMVG_LOG_DEBUG("[poseEstimation]\tRefine failed.");
    return false;
  }
  
  return true;
  
}
  
#endif //HAVE_OPENGV

// subposes is n-1 as we consider the first camera as the main camera and the 
// reference frame of the grid
bool CCTagLocalizer::localizeRig_naive(const std::vector<std::unique_ptr<features::Regions> > & vec_queryRegions,
                                 const std::vector<std::pair<std::size_t, std::size_t> > &imageSize,
                                 const LocalizerParameters *parameters,
                                 std::vector<cameras::Pinhole_Intrinsic_Radial_K3 > &vec_queryIntrinsics,
                                 const std::vector<geometry::Pose3 > &vec_subPoses,
                                 geometry::Pose3 &rigPose,
                                 std::vector<LocalizationResult>& vec_localizationResults)
{
  const CCTagLocalizer::Parameters *param = static_cast<const CCTagLocalizer::Parameters *>(parameters);
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
    isLocalized[i] = localize(vec_queryRegions[i], imageSize[i], param, true /*useInputIntrinsics*/, vec_queryIntrinsics[i], vec_localizationResults[i]);
    if(!isLocalized[i])
    {
      OPENMVG_CERR("Could not localize camera " << i);
      // even if it is not localize we can try to go on and do with the cameras we have
    }
  }
  
  // ** 'easy' cases in which we don't need further processing **
  
  const std::size_t numLocalizedCam = std::count(isLocalized.begin(), isLocalized.end(), true);
  
  // no camera has be localized
  if(numLocalizedCam == 0)
  {
    OPENMVG_LOG_DEBUG("No camera has been localized!!!");
    return false;
  }
  
  OPENMVG_LOG_DEBUG("Localized cameras: " << numLocalizedCam << "/" << numCams);
  
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
    
    OPENMVG_LOG_DEBUG("Index of the first localized camera: " << idx);
    
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
    OPENMVG_LOG_DEBUG("[poseEstimation]\tRig pose refinement failed.");
    return false;
  }
  
  updateRigPoses(vec_localizationResults, rigPose, vec_subPoses);
  
  return true;
}


void CCTagLocalizer::getAllAssociations(const features::CCTAG_Regions &queryRegions,
                                        const std::pair<std::size_t, std::size_t> &imageSize,
                                        const CCTagLocalizer::Parameters &param,
                                        std::map< std::pair<IndexT, IndexT>, std::size_t > &occurences, 
                                        Mat &pt2D,
                                        Mat &pt3D,
                                        std::vector<voctree::DocMatch>& matchedImages,
                                        const std::string& imagePath) const
{
  std::vector<IndexT> nearestKeyFrames;
  nearestKeyFrames.reserve(param._nNearestKeyFrames);
  
  kNearestKeyFrames(queryRegions,
                    _regions_per_view,
                    param._nNearestKeyFrames,
                    nearestKeyFrames);
  
  matchedImages.clear();
  matchedImages.reserve(nearestKeyFrames.size());
  
  OPENMVG_LOG_DEBUG("nearestKeyFrames.size() = " << nearestKeyFrames.size());
  for(const IndexT indexKeyFrame : nearestKeyFrames)
  {
    OPENMVG_LOG_DEBUG(indexKeyFrame);
    OPENMVG_LOG_DEBUG(_sfm_data.GetViews().at(indexKeyFrame)->s_Img_path);
    const Reconstructed_RegionsCCTag& matchedRegions = _regions_per_view.at(indexKeyFrame);
    
    // Matching
    std::vector<matching::IndMatch> vec_featureMatches;
    viewMatching(queryRegions, _regions_per_view.at(indexKeyFrame)._regions, vec_featureMatches);
    OPENMVG_LOG_DEBUG("matching]\tFound "<< vec_featureMatches.size() <<" matches.");
    
    matchedImages.emplace_back(indexKeyFrame, vec_featureMatches.size());
    
    if(!param._visualDebug.empty() && !imagePath.empty())
    {
      namespace bfs = boost::filesystem;
      const sfm::View *mview = _sfm_data.GetViews().at(indexKeyFrame).get();
      const std::string queryImage = bfs::path(imagePath).stem().string();
      const std::string matchedImage = bfs::path(mview->s_Img_path).stem().string();
      const std::string matchedPath = (bfs::path(_sfm_data.s_root_path) /  bfs::path(mview->s_Img_path)).string();

      // the directory where to save the feature matches
      const auto baseDir = bfs::path(param._visualDebug) / queryImage;
      if((!bfs::exists(baseDir)))
      {
        OPENMVG_LOG_DEBUG("created " << baseDir.string());
        bfs::create_directories(baseDir);
      }
      
      // the final filename for the output svg file as a composition of the query
      // image and the matched image
      auto outputName = baseDir / queryImage;
      outputName += "_";
      outputName += matchedImage;
      outputName += ".svg";
      
      const bool showNotMatched = true;
      features::saveCCTagMatches2SVG(imagePath, 
                                     imageSize, 
                                     queryRegions,
                                     matchedPath,
                                     std::make_pair(mview->ui_width, mview->ui_height), 
                                     _regions_per_view.at(indexKeyFrame)._regions,
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
      const IndexT pt3D_id = matchedRegions._associated3dPoint[featureMatch._j];
      const IndexT pt2D_id = featureMatch._i;
      
      const auto key = std::make_pair(pt3D_id, pt2D_id);
      if(occurences.count(key))
      {
        occurences[key]++;
      }
      else
      {
        occurences[key] = 1;
      }
    }
  }
      
  const size_t numCollectedPts = occurences.size();
  OPENMVG_LOG_DEBUG("[matching]\tCollected "<< numCollectedPts <<" associations.");
  
  {
    // just debugging statistics, this block can be safely removed    
    std::size_t maxOcc = 0;
    for(const auto &idx : occurences)
    {
      const auto &key = idx.first;
      const auto &value = idx.second;
       OPENMVG_LOG_DEBUG("[matching]\tAssociations "
               << key.first << "," << key.second <<"] found " 
               << value << " times.");
       if(value > maxOcc)
         maxOcc = value;
    }
    
    std::size_t numOccTreated = 0;
    for(std::size_t value = 1; value < maxOcc; ++value)
    {
      std::size_t counter = 0;
      for(const auto &idx : occurences)
      {
        if(idx.second == value)
        {
          ++counter;
        }
      }
      if(counter>0)
        OPENMVG_LOG_DEBUG("[matching]\tThere are " << counter
                    << " associations occurred " << value << " times ("
                    << 100.0 * counter / (double) numCollectedPts << "%)");
      numOccTreated += counter;
      if(numOccTreated >= numCollectedPts)
        break;
    }
  }

  pt2D = Mat2X(2, numCollectedPts);
  pt3D = Mat3X(3, numCollectedPts);
      
  size_t index = 0;
  for(const auto &idx : occurences)
  {
    // recopy all the points in the matching structure
    const IndexT pt3D_id = idx.first.first;
    const IndexT pt2D_id = idx.first.second;
      
    pt2D.col(index) = queryRegions.GetRegionPosition(pt2D_id);
    pt3D.col(index) = _sfm_data.GetLandmarks().at(pt3D_id).X;
      ++index;
  }
}





void kNearestKeyFrames(const features::CCTAG_Regions & queryRegions,
                       const CCTagRegionsPerViews & regionsPerView,
                       std::size_t nNearestKeyFrames,
                       std::vector<IndexT> & kNearestFrames,
                       const float similarityThreshold /*=.0f*/)
{
  kNearestFrames.clear();
  
  // A std::multimap is used instead of a std::map because is very likely that the
  // similarity measure is equal for a subset of views in the CCTag regions case.
  std::multimap<float, IndexT> sortedViewSimilarities;
  
  for(const auto & keyFrame : regionsPerView)
  {
    const float similarity = viewSimilarity(queryRegions, keyFrame.second._regions);
    sortedViewSimilarities.emplace(similarity, keyFrame.first);
  }
  
  std::size_t counter = 0;
  kNearestFrames.reserve(nNearestKeyFrames);
  for (auto rit = sortedViewSimilarities.crbegin(); rit != sortedViewSimilarities.crend(); ++rit)
  {
    if(rit->first < similarityThreshold)
      // since it is ordered, the first having smaller similarity guarantees that
      // there won't be other useful kframes
      break;
    
    kNearestFrames.push_back(rit->second);
    ++counter;
    
    if (counter == nNearestKeyFrames)
      break;
  }
}
 
void viewMatching(const features::CCTAG_Regions & regionsA,
                  const features::CCTAG_Regions & regionsB,
                  std::vector<matching::IndMatch> & vec_featureMatches)
{
  vec_featureMatches.clear();
  
  for(std::size_t i=0 ; i < regionsA.Descriptors().size() ; ++i)
  {
    const IndexT cctagIdA = features::getCCTagId(regionsA.Descriptors()[i]);
    // todo: Should be change to: Find in regionsB.Descriptors() the nearest 
    // descriptor to descriptorA. Currently, a cctag descriptor encode directly
    // the cctag id, then the id equality is tested.
    for(std::size_t j=0 ; j < regionsB.Descriptors().size() ; ++j)
    {
      const IndexT cctagIdB = features::getCCTagId(regionsB.Descriptors()[j]);
      if ( cctagIdA == cctagIdB )
      {
        vec_featureMatches.emplace_back(i,j);
        break;
      }
    }
  }
}
 
 
 
float viewSimilarity(const features::CCTAG_Regions & regionsA,
                     const features::CCTAG_Regions & regionsB)
{
  assert(regionsA.DescriptorLength() == regionsB.DescriptorLength()); 
  
  const std::bitset<128> descriptorViewA = constructCCTagViewDescriptor(regionsA.Descriptors());
  const std::bitset<128> descriptorViewB = constructCCTagViewDescriptor(regionsB.Descriptors());
  
  // The similarity is the sum of all the cctags sharing the same id visible in both views.
  return (descriptorViewA & descriptorViewB).count();
}

std::bitset<128> constructCCTagViewDescriptor(const std::vector<CCTagDescriptor> & vCCTagDescriptors)
{
  std::bitset<128> descriptorView;
  for(const auto & cctagDescriptor : vCCTagDescriptors )
  {
    const IndexT cctagId = features::getCCTagId(cctagDescriptor);
    if ( cctagId != UndefinedIndexT)
    {
      descriptorView.set(cctagId, true);
    }
  }
  return descriptorView;
}

} // localization
} // openMVG

