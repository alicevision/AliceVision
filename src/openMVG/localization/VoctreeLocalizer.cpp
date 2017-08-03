#include "VoctreeLocalizer.hpp"
#include "rigResection.hpp"
#include "optimization.hpp"
#include <openMVG/config.hpp>
#include <openMVG/sfm/sfm_data_io.hpp>
#include <openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp>
#include <openMVG/sfm/sfm_data_BA_ceres.hpp>
#include <openMVG/sfm/pipelines/RegionsIO.hpp>
#include <openMVG/features/io_regions_type.hpp>
#include <openMVG/features/svgVisualization.hpp>
#include <openMVG/matching/regions_matcher.hpp>
#include <openMVG/matching_image_collection/Matcher.hpp>
#include <openMVG/matching_image_collection/GeometricFilterMatrix.hpp>
#include <openMVG/matching/matcher_kdtree_flann.hpp>
#include <openMVG/matching_image_collection/F_ACRobust.hpp>
#include <openMVG/matching_image_collection/GeometricFilterMatrix.hpp>
#include <openMVG/numeric/numeric.h>
#include <openMVG/robust_estimation/guided_matching.hpp>
#include <openMVG/logger.hpp>
#include <openMVG/system/timer.hpp>

#include <third_party/progress/progress.hpp>

#include <boost/filesystem.hpp>

#include <algorithm>
#include <chrono>

namespace openMVG {
namespace localization {

std::ostream& operator<<( std::ostream& os, const voctree::Document &doc )	
{
  os << "[ ";
  for( const voctree::Word &w : doc )
  {
          os << w << ", ";
  }
  os << "];\n";
  return os;
}

std::ostream& operator<<(std::ostream& os, VoctreeLocalizer::Algorithm a)
{
  switch(a)
  {
  case VoctreeLocalizer::Algorithm::FirstBest: os << "FirstBest";
    break;
  case VoctreeLocalizer::Algorithm::BestResult: os << "BestResult";
    break;
  case VoctreeLocalizer::Algorithm::AllResults: os << "AllResults";
    break;
  case VoctreeLocalizer::Algorithm::Cluster: os << "Cluster";
    break;
  default: 
    os << "Unknown algorithm!";
    throw std::invalid_argument("Unrecognized algorithm!");
  }
  return os;
}

std::istream& operator>>(std::istream &in, VoctreeLocalizer::Algorithm &a)
{
  int i;
  in >> i;
  a = static_cast<VoctreeLocalizer::Algorithm> (i);
  return in;
}	

FrameData::FrameData(const LocalizationResult &locResult, const features::MapRegionsPerDesc& regionsPerDesc)
  : _locResult(locResult)
{
  // now we need to filter out and keep only the regions with 3D data associated
  const auto &associationIDs = _locResult.getIndMatch3D2D();
  const auto &inliers = _locResult.getInliers();

  // Regions for each describer type
  for(const auto& regionsPerDescIt : regionsPerDesc)
  {
    // feature in image are <featureID, point3Did> associations
    std::vector<features::FeatureInImage> featuresInImage;
    featuresInImage.reserve(inliers.size());

    assert(inliers.size() <= associationIDs.size());
    for(const auto &idx : inliers)
    {
      assert(idx < associationIDs.size());
      const auto &association = associationIDs[idx];
      assert(association.descType != features::EImageDescriberType::UNINITIALIZED);
      if(association.descType != regionsPerDescIt.first)
        continue;

     // add it to featuresInImage
      featuresInImage.emplace_back(association.featId, association.landmarkId);
    }

    _regions[regionsPerDescIt.first] = createFilteredRegions(*regionsPerDescIt.second, featuresInImage, _regionsWith3D[regionsPerDescIt.first]);
  }
}

VoctreeLocalizer::Algorithm VoctreeLocalizer::initFromString(const std::string &value)
{
  if(value=="FirstBest")
    return VoctreeLocalizer::Algorithm::FirstBest;
  else if(value=="AllResults")
    return VoctreeLocalizer::Algorithm::AllResults;
  else if(value=="BestResult")
    throw std::invalid_argument("BestResult not yet implemented");
  else if(value=="Cluster")
    throw std::invalid_argument("Cluster not yet implemented");
  else
    throw std::invalid_argument("Unrecognized algorithm \"" + value + "\"!");
}


VoctreeLocalizer::VoctreeLocalizer(const std::string &sfmFilePath,
                                   const std::string &descriptorsFolder,
                                   const std::string &vocTreeFilepath,
                                   const std::string &weightsFilepath,
                                   const std::vector<features::EImageDescriberType>& matchingDescTypes)
  : ILocalizer()
  , _frameBuffer(5)
{
  using namespace openMVG::features;

  // init the feature extractor
  _imageDescribers.reserve(matchingDescTypes.size());
  for(features::EImageDescriberType matchingDescType: matchingDescTypes)
  {
    _imageDescribers.push_back(createImageDescriber(matchingDescType));
  }

  // load the sfm data containing the 3D reconstruction info
  OPENMVG_LOG_DEBUG("Loading SFM data...");
  if (!Load(_sfm_data, sfmFilePath, sfm::ESfM_Data::ALL)) 
  {
    OPENMVG_CERR("The input SfM_Data file " << sfmFilePath << " cannot be read!");
    OPENMVG_CERR("\n\nIf the error says \"JSON Parsing failed - provided NVP not found\" "
            "it's likely that you have to convert your sfm_data to a recent version supporting "
            "polymorphic Views. You can run the python script convertSfmData.py to update an existing sfmdata.");
    _isInit = false;
    return;
  }

  OPENMVG_LOG_DEBUG("SfM data loaded from " << sfmFilePath << " containing: ");
  OPENMVG_LOG_DEBUG("\tnumber of views      : " << _sfm_data.GetViews().size());
  OPENMVG_LOG_DEBUG("\tnumber of poses      : " << _sfm_data.GetPoses().size());
  OPENMVG_LOG_DEBUG("\tnumber of points     : " << _sfm_data.GetLandmarks().size());
  OPENMVG_LOG_DEBUG("\tnumber of intrinsics : " << _sfm_data.GetIntrinsics().size());

  // load the features and descriptors
  // initially we need all the feature in order to create the database
  // then we can store only those associated to 3D points
  //? can we use Feature_Provider to load the features and filter them later?

  const std::string descFolder = descriptorsFolder.empty() ? _sfm_data.getFeatureFolder() : descriptorsFolder;
  _isInit = initDatabase(vocTreeFilepath, weightsFilepath, descFolder);
}

bool VoctreeLocalizer::localize(const features::MapRegionsPerDesc & queryRegions,
                                const std::pair<std::size_t, std::size_t> &imageSize,
                                const LocalizerParameters *param,
                                bool useInputIntrinsics,
                                cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                                LocalizationResult & localizationResult,
                                const std::string& imagePath)
{
  const Parameters *voctreeParam = static_cast<const Parameters *>(param);
  if(!voctreeParam)
  {
    // error!
    throw std::invalid_argument("The parameters are not in the right format!!");
  }
  
  switch(voctreeParam->_algorithm)
  {
    case Algorithm::FirstBest:
    return localizeFirstBestResult(queryRegions,
                                   imageSize,
                                   *voctreeParam,
                                   useInputIntrinsics,
                                   queryIntrinsics,
                                   localizationResult,
                                   imagePath);
    case Algorithm::BestResult: throw std::invalid_argument("BestResult not yet implemented");
    case Algorithm::AllResults:
    return localizeAllResults(queryRegions,
                              imageSize,
                              *voctreeParam,
                              useInputIntrinsics,
                              queryIntrinsics,
                              localizationResult,
                              imagePath);
    case Algorithm::Cluster: throw std::invalid_argument("Cluster not yet implemented");
    default: throw std::invalid_argument("Unknown algorithm type");
  }
}

bool VoctreeLocalizer::localize(const image::Image<unsigned char> & imageGrey,
                                const LocalizerParameters *param,
                                bool useInputIntrinsics,
                                cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                                LocalizationResult &localizationResult,
                                const std::string& imagePath /* = std::string() */)
{
  // A. extract descriptors and features from image
  OPENMVG_LOG_DEBUG("[features]\tExtract Regions from query image");
  features::MapRegionsPerDesc queryRegionsPerDesc;

  for(const auto& imageDescriber : _imageDescribers)
  {
    const auto descType = imageDescriber->getDescriberType();
    auto & queryRegions = queryRegionsPerDesc[descType];

    imageDescriber->Allocate(queryRegions);

    system::Timer timer;
    imageDescriber->Set_configuration_preset(param->_featurePreset);
    imageDescriber->Describe(imageGrey, queryRegions, nullptr);

    OPENMVG_LOG_DEBUG("[features]\tExtract " << features::EImageDescriberType_enumToString(descType) << " done: found " << queryRegions->RegionCount() << " features in " << timer.elapsedMs() << " [ms]");
  }

  const std::pair<std::size_t, std::size_t> queryImageSize = std::make_pair(imageGrey.Width(), imageGrey.Height());

  // if debugging is enable save the svg image with the extracted features
  if(!param->_visualDebug.empty() && !imagePath.empty())
  {
    features::MapFeaturesPerDesc extractedFeatures;

    for(const auto& imageDescriber : _imageDescribers)
    {
      const auto descType = imageDescriber->getDescriberType();
      extractedFeatures[descType] = queryRegionsPerDesc.at(descType)->GetRegionsPositions();
    }

    namespace bfs = boost::filesystem;
    features::saveFeatures2SVG(imagePath,
                     queryImageSize,
                     extractedFeatures,
                     param->_visualDebug + "/" + bfs::path(imagePath).stem().string() + ".svg");
  }

  return localize(queryRegionsPerDesc,
                  queryImageSize,
                  param,
                  useInputIntrinsics,
                  queryIntrinsics,
                  localizationResult,
                  imagePath);
}

bool VoctreeLocalizer::loadReconstructionDescriptors(const sfm::SfM_Data & sfm_data,
                                                     const std::string & feat_directory)
{
  //@fixme deprecated: now inside initDatabase
  throw std::logic_error("loadReconstructionDescriptors is deprecated, use initDatabase instead.");
  return true;
}

/**
 * @brief Initialize the database: load features & descriptors for reconstructed landmarks,
 *        and create voctree image desc.
 */
bool VoctreeLocalizer::initDatabase(const std::string & vocTreeFilepath,
                                    const std::string & weightsFilepath,
                                    const std::string & feat_directory)
{

  bool withWeights = !weightsFilepath.empty();

  // Load vocabulary tree
  OPENMVG_LOG_DEBUG("Loading vocabulary tree...");

  voctree::load(_voctree, _voctreeDescType, vocTreeFilepath);

  OPENMVG_LOG_DEBUG("tree loaded with " << _voctree->levels() << " levels and "
          << _voctree->splits() << " branching factors");

  OPENMVG_LOG_DEBUG("Creating the database...");
  // Add each object (document) to the database
  _database = voctree::Database(_voctree->words());
  if(withWeights)
  {
    OPENMVG_LOG_DEBUG("Loading weights...");
    _database.loadWeights(weightsFilepath);
  }
  else
  {
    OPENMVG_LOG_DEBUG("No weights specified, skipping...");
  }

  // Load the descriptors and the features related to the images
  // for every image, pass the descriptors through the vocabulary tree and
  // add its visual words to the database.
  // then only store the feature and descriptors that have a 3D point associated
  OPENMVG_LOG_DEBUG("Build observations per view");
  C_Progress_display my_progress_bar(_sfm_data.GetViews().size(),
                                     std::cout, "\n- Load Features and Descriptors per view -\n");

  // Build observations per view
  std::map<IndexT, std::map<features::EImageDescriberType, std::vector<features::FeatureInImage>>> observationsPerView;
  for(const auto& landmarkValue : _sfm_data.structure)
  {
    IndexT trackId = landmarkValue.first;
    const sfm::Landmark& landmark = landmarkValue.second;

    for(const auto& obs : landmark.observations)
    {
      const IndexT viewId = obs.first;
      const sfm::Observation& obs2d = obs.second;
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

  // Read for each view the corresponding Regions and store them
#pragma omp parallel for num_threads(3)
  for(int i = 0; i < _sfm_data.GetViews().size(); ++i)
  {
    auto iter = _sfm_data.GetViews().cbegin();
    std::advance(iter, i);
    const IndexT id_view = iter->second->id_view;
    if(observationsPerView.count(id_view) == 0)
      continue;
    const auto& observations = observationsPerView.at(id_view);
    for(const auto& imageDescriber: _imageDescribers)
    {
      const features::EImageDescriberType descType = imageDescriber->getDescriberType();

      ReconstructedRegionsMapping mapping;
      if(observations.count(descType) == 0)
      {
        // no descriptor of this type in this View
#pragma omp critical
        {
          // We always initialize objects with empty structures,
          // so all views and descTypes always exist in the map.
          // It simplifies the code based on these data structures,
          // so you have a data structure with 0 element and you don't need to add
          // special cases everywhere for empty elements.
          _reconstructedRegionsMappingPerView[id_view][descType] = std::move(mapping);
          imageDescriber->Allocate(_regionsPerView.getData()[id_view][descType]);
        }
        continue;
      }

      // Load from files
      std::unique_ptr<features::Regions> currRegions = sfm::loadRegions(feat_directory, id_view, *imageDescriber);

      if(descType == _voctreeDescType)
      {
        voctree::SparseHistogram histo = _voctree->quantizeToSparse(currRegions->blindDescriptors());
#pragma omp critical
        {
          _database.insert(id_view, histo);
        }
      }


      // Filter descriptors to keep only the 3D reconstructed points
      std::unique_ptr<features::Regions> filteredRegions = createFilteredRegions(*currRegions, observations.at(descType), mapping);
#pragma omp critical
      {
        _reconstructedRegionsMappingPerView[id_view][descType] = std::move(mapping);
        _regionsPerView.getData()[id_view][descType] = std::move(filteredRegions);
      }
    }
#pragma omp critical
    {
      ++my_progress_bar;
    }
  }
  return true;
}

bool VoctreeLocalizer::localizeFirstBestResult(const features::MapRegionsPerDesc &queryRegions,
                                               const std::pair<std::size_t, std::size_t> queryImageSize,
                                               const Parameters &param,
                                               bool useInputIntrinsics,
                                               cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                                               LocalizationResult &localizationResult,
                                               const std::string& imagePath)
{
  // A. Find the (visually) similar images in the database 
  OPENMVG_LOG_DEBUG("[database]\tRequest closest images from voctree");
  // pass the descriptors through the vocabulary tree to get the visual words
  // associated to each feature
  voctree::SparseHistogram requestImageWords = _voctree->quantizeToSparse(queryRegions.at(_voctreeDescType)->blindDescriptors());
  
  // Request closest images from voctree
  std::vector<voctree::DocMatch> matchedImages;
  _database.find(requestImageWords, param._numResults, matchedImages);
  
//  // Debugging log
//  // for each similar image found print score and number of features
//  for(const voctree::DocMatch & currMatch : matchedImages )
//  {
//    // get the corresponding index of the view
//    const IndexT matchedViewIndex = currMatch.id;
//    // get the view handle
//    const std::shared_ptr<sfm::View> matchedView = _sfm_data.views[matchedViewIndex];
//    OPENMVG_LOG_DEBUG( "[database]\t\t match " << matchedView->s_Img_path 
//            << " [docid: "<< currMatch.id << "]"
//            << " with score " << currMatch.score 
//            << " and it has "  << _regions_per_view[matchedViewIndex]._regions.RegionCount() 
//            << " features with 3D points");
//  }

  OPENMVG_LOG_DEBUG("[matching]\tBuilding the matcher");
  matching::RegionsDatabaseMatcherPerDesc matchers(_matcherType, queryRegions);

  sfm::Image_Localizer_Match_Data resectionData;
  std::vector<IndMatch3D2D> associationIDs;
  geometry::Pose3 pose;
 
  // B. for each found similar image, try to find the correspondences between the 
  // query image and the similar image
  for(const voctree::DocMatch& matchedImage : matchedImages)
  {
    // minimum number of points that allows a reliable 3D reconstruction
    const size_t minNum3DPoints = 5;
    
    // the view index of the current matched image
    const IndexT matchedViewId = matchedImage.id;
    // the handler to the current view
    const std::shared_ptr<sfm::View> matchedView = _sfm_data.views.at(matchedViewId);

    // safeguard: we should match the query image with an image that has at least
    // some 3D points visible --> if it has 0 3d points it is likely that it is an
    // image of the dataset that was not reconstructed
    if(_regionsPerView.getRegionsPerDesc(matchedViewId).getNbAllRegions() < minNum3DPoints)
    {
      OPENMVG_LOG_DEBUG("[matching]\tSkipping matching with " << matchedView->s_Img_path << " as it has too few visible 3D points (" << _regionsPerView.getRegionsPerDesc(matchedViewId).getNbAllRegions() << ")");
      continue;
    }
    OPENMVG_LOG_DEBUG("[matching]\tTrying to match the query image with " << matchedView->s_Img_path);

    // its associated intrinsics
    const cameras::IntrinsicBase *matchedIntrinsicsBase = _sfm_data.GetIntrinsicPtr(matchedView->id_intrinsic);
    if ( !isPinhole(matchedIntrinsicsBase->getType()) )
      throw std::logic_error("Unsupported intrinsic: " + EINTRINSIC_enumToString(matchedIntrinsicsBase->getType()) + " (only Pinhole cameras are supported for localization).");

    const cameras::Pinhole_Intrinsic *matchedIntrinsics = (const cameras::Pinhole_Intrinsic*)(matchedIntrinsicsBase);
    
    matching::MatchesPerDescType featureMatches;
    bool matchWorked = robustMatching(matchers,
                                      // pass the input intrinsic if they are valid, null otherwise
                                      (useInputIntrinsics) ? &queryIntrinsics : nullptr,
                                      _regionsPerView.getRegionsPerDesc(matchedViewId),
                                      matchedIntrinsics,
                                      param._fDistRatio,
                                      param._matchingError,
                                      param._useRobustMatching,
                                      param._useGuidedMatching,
                                      queryImageSize,
                                      std::make_pair(matchedView->ui_width, matchedView->ui_height), 
                                      featureMatches,
                                      param._matchingEstimator);
    if (!matchWorked)
    {
      OPENMVG_LOG_DEBUG("[matching]\tMatching with " << matchedView->s_Img_path << " failed! Skipping image");
      continue;
    }

    std::size_t nbAllMatches = featureMatches.getNbAllMatches();
    OPENMVG_LOG_DEBUG("[matching]\tFound " << nbAllMatches << " geometrically validated matches");
    assert(nbAllMatches > 0);
    
    if(!param._visualDebug.empty() && !imagePath.empty())
    {
      namespace bfs = boost::filesystem;
      const sfm::View *mview = _sfm_data.GetViews().at(matchedViewId).get();
      const std::string queryimage = bfs::path(imagePath).stem().string();
      const std::string matchedImage = bfs::path(mview->s_Img_path).stem().string();
      const std::string matchedPath = (bfs::path(_sfm_data.s_root_path) /  bfs::path(mview->s_Img_path)).string();
      
      features::saveMatches2SVG(imagePath,
                      queryImageSize,
                      queryRegions,
                      matchedPath,
                      std::make_pair(mview->ui_width, mview->ui_height),
                      _regionsPerView.getRegionsPerDesc(matchedViewId),
                      featureMatches,
                      param._visualDebug + "/" + queryimage + "_" + matchedImage + ".svg"); 
    }
    
    // C. recover the 2D-3D associations from the matches 
    // Each matched feature in the current similar image is associated to a 3D point,
    // hence we can recover the 2D-3D associations to estimate the pose
    // Prepare data for resection
    resectionData = sfm::Image_Localizer_Match_Data();
    resectionData.pt2D = Mat2X(2, nbAllMatches);
    resectionData.pt3D = Mat3X(3, nbAllMatches);
    associationIDs.clear();
    associationIDs.reserve(nbAllMatches);

    // Get the 3D points associated to each matched feature
    std::size_t index = 0;
    for(const auto& featureMatchesIt : featureMatches)
    {
      const features::EImageDescriberType descType = featureMatchesIt.first;
      const auto& matchedRegions = _reconstructedRegionsMappingPerView.at(matchedViewId).at(descType);

      for(const matching::IndMatch& featureMatch : featureMatchesIt.second)
      {
        // the ID of the 3D point
        const IndexT trackId3D = matchedRegions._associated3dPoint[featureMatch._j];

        // prepare data for resectioning
        resectionData.pt3D.col(index) = _sfm_data.GetLandmarks().at(trackId3D).X;

        const Vec2 feat = queryRegions.at(descType)->GetRegionPosition(featureMatch._i);
        resectionData.pt2D.col(index) = feat;

        associationIDs.emplace_back(trackId3D, descType, featureMatch._i);

        ++index;
      }
    }
    assert(index == nbAllMatches);

    // estimate the pose
    // Do the resectioning: compute the camera pose.
    resectionData.error_max = param._errorMax;
    OPENMVG_LOG_DEBUG("[poseEstimation]\tEstimating camera pose...");
    bool bResection = sfm::SfM_Localizer::Localize(queryImageSize,
                                                   // pass the input intrinsic if they are valid, null otherwise
                                                   (useInputIntrinsics) ? &queryIntrinsics : nullptr,
                                                   resectionData,
                                                   pose,
                                                   param._resectionEstimator);

    if(!bResection)
    {
      OPENMVG_LOG_DEBUG("[poseEstimation]\tResection failed");
      // try next one
      continue;
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
      OPENMVG_LOG_DEBUG("K estimated\n" << K_);
      queryIntrinsics.setK(K_);
      queryIntrinsics.setWidth(queryImageSize.first);
      queryIntrinsics.setHeight(queryImageSize.second);
    }

    // D. refine the estimated pose
    OPENMVG_LOG_DEBUG("[poseEstimation]\tRefining estimated pose");
    bool refineStatus = sfm::SfM_Localizer::RefinePose(&queryIntrinsics, 
                                                       pose, 
                                                       resectionData, 
                                                       true /*b_refine_pose*/, 
                                                       param._refineIntrinsics /*b_refine_intrinsic*/);
    if(!refineStatus)
    {
      OPENMVG_LOG_DEBUG("[poseEstimation]\tRefine pose failed.");
      // try next one
      continue;
    }
    
    {
      // just temporary code to evaluate the estimated pose @todo remove it
      const geometry::Pose3 &referencePose = _sfm_data.poses[matchedViewId];
      OPENMVG_LOG_DEBUG("R refined\n" << pose.rotation());
      OPENMVG_LOG_DEBUG("t refined\n" << pose.translation());
      OPENMVG_LOG_DEBUG("K refined\n" << queryIntrinsics.K());
      OPENMVG_LOG_DEBUG("R_gt\n" << referencePose.rotation());
      OPENMVG_LOG_DEBUG("t_gt\n" << referencePose.translation());
      OPENMVG_LOG_DEBUG("angular difference: " << R2D(getRotationMagnitude(pose.rotation()*referencePose.rotation().inverse())) << "deg");
      OPENMVG_LOG_DEBUG("center difference: " << (pose.center()-referencePose.center()).norm());
      OPENMVG_LOG_DEBUG("err = [err; " << R2D(getRotationMagnitude(pose.rotation()*referencePose.rotation().inverse())) << ", "<< (pose.center()-referencePose.center()).norm() << "];");
    }
    localizationResult = LocalizationResult(resectionData, associationIDs, pose, queryIntrinsics, matchedImages, refineStatus);
    break;
  }
  //@todo deal with unsuccesful case...
  return localizationResult.isValid();
  
 } 

bool VoctreeLocalizer::localizeAllResults(const features::MapRegionsPerDesc &queryRegions,
                                          const std::pair<std::size_t, std::size_t> queryImageSize,
                                          const Parameters &param,
                                          bool useInputIntrinsics,
                                          cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                                          LocalizationResult &localizationResult,
                                          const std::string& imagePath)
{
  
  sfm::Image_Localizer_Match_Data resectionData;
  // a map containing for each pair <pt3D_id, pt2D_id> the number of times that 
  // the association has been seen
  OccurenceMap occurences;
  
  // get all the association from the database images
  std::vector<voctree::DocMatch> matchedImages;
  getAllAssociations(queryRegions,
                     queryImageSize,
                     param,
                     useInputIntrinsics,
                     queryIntrinsics,
                     occurences,
                     resectionData.pt2D,
                     resectionData.pt3D,
                     resectionData.vec_descType,
                     matchedImages,
                     imagePath);

  const std::size_t numCollectedPts = occurences.size();
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
  
  // estimate the pose
  // Do the resectioning: compute the camera pose.
  resectionData.error_max = param._errorMax;
  OPENMVG_LOG_DEBUG("[poseEstimation]\tEstimating camera pose...");
  const bool bResection = sfm::SfM_Localizer::Localize(queryImageSize,
                                                      // pass the input intrinsic if they are valid, null otherwise
                                                      (useInputIntrinsics) ? &queryIntrinsics : nullptr,
                                                      resectionData,
                                                      pose,
                                                      param._resectionEstimator);

  if(!bResection)
  {
    OPENMVG_LOG_DEBUG("[poseEstimation]\tResection failed");
    if(!param._visualDebug.empty() && !imagePath.empty())
    {
      namespace bfs = boost::filesystem;
      features::saveFeatures2SVG(imagePath,
                                 queryImageSize,
                                 resectionData.pt2D,
                                 param._visualDebug + "/" + bfs::path(imagePath).stem().string() + ".associations.svg");
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
    queryIntrinsics.setWidth(queryImageSize.first);
    queryIntrinsics.setHeight(queryImageSize.second);
  }

  // E. refine the estimated pose
  OPENMVG_LOG_DEBUG("[poseEstimation]\tRefining estimated pose");
  bool refineStatus = sfm::SfM_Localizer::RefinePose(&queryIntrinsics,
                                                     pose,
                                                     resectionData,
                                                     true /*b_refine_pose*/,
                                                     param._refineIntrinsics /*b_refine_intrinsic*/);
  if(!refineStatus)
    OPENMVG_LOG_DEBUG("Refine pose failed.");

  if(!param._visualDebug.empty() && !imagePath.empty())
  {
    namespace bfs = boost::filesystem;
    features::saveFeatures2SVG(imagePath,
                     queryImageSize,
                     resectionData.pt2D,
                     param._visualDebug + "/" + bfs::path(imagePath).stem().string() + ".associations.svg",
                     &resectionData.vec_inliers);
  }

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

  if(param._nbFrameBufferMatching > 0)
  {
    // add everything to the buffer
    _frameBuffer.emplace_back(localizationResult, queryRegions);
  }

  return localizationResult.isValid();
}

void VoctreeLocalizer::getAllAssociations(const features::MapRegionsPerDesc &queryRegions,
                                          const std::pair<std::size_t, std::size_t> &imageSize,
                                          const Parameters &param,
                                          bool useInputIntrinsics,
                                          const cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                                          OccurenceMap &out_occurences,
                                          Mat &out_pt2D,
                                          Mat &out_pt3D,
                                          std::vector<features::EImageDescriberType>& out_descTypes,
                                          std::vector<voctree::DocMatch>& out_matchedImages,
                                          const std::string& imagePath) const
{
  assert(out_descTypes.size() == 0);

  // A. Find the (visually) similar images in the database 
  // pass the descriptors through the vocabulary tree to get the visual words
  // associated to each feature
  OPENMVG_LOG_DEBUG("[database]\tRequest closest images from voctree");
  if(queryRegions.count(_voctreeDescType) == 0)
  {
    OPENMVG_LOG_WARNING("[database]\t No feature type " << features::EImageDescriberType_enumToString(_voctreeDescType) << " in query region.");
    return;
  }
  voctree::SparseHistogram requestImageWords = _voctree->quantizeToSparse(queryRegions.at(_voctreeDescType)->blindDescriptors());
  
  // Request closest images from voctree
  _database.find(requestImageWords, (param._numResults==0) ? (_database.size()) : (param._numResults) , out_matchedImages);

//  // Debugging log
//  // for each similar image found print score and number of features
//  for(const voctree::DocMatch& currMatch : matchedImages )
//  {
//    // get the view handle
//    const std::shared_ptr<sfm::View> matchedView = _sfm_data.views[currMatch.id];
//    OPENMVG_LOG_DEBUG( "[database]\t\t match " << matchedView->s_Img_path 
//            << " [docid: "<< currMatch.id << "]"
//            << " with score " << currMatch.score 
//            << " and it has "  << _regions_per_view[currMatch.id]._regions.RegionCount() 
//            << " features with 3D points");
//  }

  OPENMVG_LOG_DEBUG("[matching]\tBuilding the matcher");
  matching::RegionsDatabaseMatcherPerDesc matchers(_matcherType, queryRegions);

  std::map< std::pair<IndexT, IndexT>, std::size_t > repeated;
  
  // B. for each found similar image, try to find the correspondences between the 
  // query image adn the similar image
  // stop when param._maxResults successful matches have been found
  std::size_t goodMatches = 0;
  for(const voctree::DocMatch& matchedImage : out_matchedImages)
  {
    // minimum number of points that allows a reliable 3D reconstruction
    const size_t minNum3DPoints = 5;

    const auto matchedViewId = matchedImage.id;
    // the handler to the current view
    const std::shared_ptr<sfm::View> matchedView = _sfm_data.views.at(matchedViewId);
    // its associated reconstructed regions
    const features::MapRegionsPerDesc& matchedRegions = _regionsPerView.getRegionsPerDesc(matchedViewId);
    
    // safeguard: we should match the query image with an image that has at least
    // some 3D points visible --> if this is not true it is likely that it is an
    // image of the dataset that was not reconstructed
    if(matchedRegions.getNbAllRegions() < minNum3DPoints)
    {
      OPENMVG_LOG_DEBUG("[matching]\tSkipping matching with " << matchedView->s_Img_path << " as it has too few visible 3D points");
      continue;
    }
    OPENMVG_LOG_TRACE("[matching]\tTrying to match the query image with " << matchedView->s_Img_path);
    OPENMVG_LOG_TRACE("[matching]\tIt has " << matchedRegions.getNbAllRegions() << " available features to match");
    
    // its associated intrinsics
    // this is just ugly!
    const cameras::IntrinsicBase *matchedIntrinsicsBase = _sfm_data.intrinsics.at(matchedView->id_intrinsic).get();
    if ( !isPinhole(matchedIntrinsicsBase->getType()) )
    {
      //@fixme maybe better to throw something here
      OPENMVG_CERR("Only Pinhole cameras are supported!");
      return;
    }
    const cameras::Pinhole_Intrinsic *matchedIntrinsics = (const cameras::Pinhole_Intrinsic*)(matchedIntrinsicsBase);

    matching::MatchesPerDescType featureMatches;
    const bool matchWorked = robustMatching(matchers,
                                      // pass the input intrinsic if they are valid, null otherwise
                                      (useInputIntrinsics) ? &queryIntrinsics : nullptr,
                                      matchedRegions,
                                      matchedIntrinsics,
                                      param._fDistRatio,
                                      param._matchingError,
                                      param._useRobustMatching,
                                      param._useGuidedMatching,
                                      imageSize,
                                      std::make_pair(matchedView->ui_width, matchedView->ui_height), 
                                      featureMatches,
                                      param._matchingEstimator);
    if (!matchWorked)
    {
//      OPENMVG_LOG_DEBUG("[matching]\tMatching with " << matchedView->s_Img_path << " failed! Skipping image");
      continue;
    }

    OPENMVG_LOG_DEBUG("[matching]\tFound " << featureMatches.getNbAllMatches() << " geometrically validated matches");
    assert(featureMatches.getNbAllMatches() > 0);

    // if debug is enable save the matches between the query image and the current matching image
    // It saves the feature matches in a folder with the same name as the query
    // image, if it does not exist it will create it. The final svg file will have
    // a name like this: queryImage_matchedImage.svg placed in the following directory:
    // param._visualDebug/queryImage/
    if(!param._visualDebug.empty() && !imagePath.empty())
    {
      namespace bfs = boost::filesystem;
      const sfm::View *mview = _sfm_data.GetViews().at(matchedViewId).get();
      // the current query image without extension
      const auto queryImage = bfs::path(imagePath).stem();
      // the matching image without extension
      const auto matchedImage = bfs::path(mview->s_Img_path).stem();
      // the full path of the matching image
      const auto matchedPath = (bfs::path(_sfm_data.s_root_path) /  bfs::path(mview->s_Img_path)).string();

      // the directory where to save the feature matches
      const auto baseDir = bfs::path(param._visualDebug) / queryImage;
      if((!bfs::exists(baseDir)))
      {
        OPENMVG_LOG_DEBUG("created " << baseDir.string());
        bfs::create_directories(baseDir);
      }
      
      // damn you, boost, what does it take to make the operator "+"?
      // the final filename for the output svg file as a composition of the query
      // image and the matched image
      auto outputName = baseDir / queryImage;
      outputName += "_";
      outputName += matchedImage;
      outputName += ".svg";

      features::saveMatches2SVG(imagePath,
                                imageSize,
                                queryRegions,
                                matchedPath,
                                std::make_pair(mview->ui_width, mview->ui_height),
                                _regionsPerView.getRegionsPerDesc(matchedViewId),
                                featureMatches,
                                outputName.string()); 
    }

    const auto& matchedRegionsMapping = _reconstructedRegionsMappingPerView.at(matchedViewId);

    // C. recover the 2D-3D associations from the matches 
    // Each matched feature in the current similar image is associated to a 3D point
    for(const auto& featureMatchesIt : featureMatches)
    {
      features::EImageDescriberType descType = featureMatchesIt.first;
      const auto& matchedRegionsMappingType = matchedRegionsMapping.at(descType);
      for(const matching::IndMatch& featureMatch : featureMatchesIt.second)
      {
        // the ID of the 3D point
        const IndexT pt3D_id = matchedRegionsMappingType._associated3dPoint[featureMatch._j];
        const IndexT pt2D_id = featureMatch._i;

        const OccurenceKey key(pt3D_id, descType, pt2D_id);
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
    ++goodMatches;
    if((param._maxResults !=0) && (goodMatches == param._maxResults))
    { 
      // let's say we have enough features
      OPENMVG_LOG_DEBUG("[matching]\tgot enough point from " << param._maxResults << " images");
      break;
    }
  }
  
  if(param._nbFrameBufferMatching > 0)
  {
    OPENMVG_LOG_DEBUG("[matching]\tUsing frameBuffer matching: matching with the past " 
            << param._nbFrameBufferMatching << " frames" );
    getAssociationsFromBuffer(matchers, imageSize, param, useInputIntrinsics, queryIntrinsics, out_occurences);
  }
  
  const std::size_t numCollectedPts = out_occurences.size();
  
  {
    // just debugging statistics, this block can be safely removed
    std::size_t numOccTreated = 0;
    for(std::size_t value = 1; value < numCollectedPts; ++value)
    {
      std::size_t counter = 0;
      for(const auto &idx : out_occurences)
      {
        if(idx.second == value)
        {
          ++counter;
        }
      }
      OPENMVG_LOG_DEBUG("[matching]\tThere are " << counter 
              <<  " associations occurred " << value << " times ("
              << 100.0*counter/(double)numCollectedPts << "%)");
      numOccTreated += counter;
      if(numOccTreated >= numCollectedPts) 
        break;
    }
  }
  
  out_pt2D = Mat2X(2, numCollectedPts);
  out_pt3D = Mat3X(3, numCollectedPts);
  

  out_descTypes.resize(out_occurences.size());

  std::size_t index = 0;
  for(const auto &idx : out_occurences)
  {
     // recopy all the points in the matching structure
    const IndexT pt2D_id = idx.first.featId;
    const sfm::Landmark& landmark = _sfm_data.GetLandmarks().at(idx.first.landmarkId);
    
//    OPENMVG_LOG_DEBUG("[matching]\tAssociation [" << idx.first.landmarkId << "," << pt2D_id << "] occurred " << idx.second << " times");

    out_pt2D.col(index) = queryRegions.at(idx.first.descType)->GetRegionPosition(pt2D_id);
    out_pt3D.col(index) = landmark.X;
    out_descTypes.at(index) = landmark.descType;
     ++index;
  }
  
}

void VoctreeLocalizer::getAssociationsFromBuffer(matching::RegionsDatabaseMatcherPerDesc & matchers,
                                                 const std::pair<std::size_t, std::size_t> queryImageSize,
                                                 const Parameters &param,
                                                 bool useInputIntrinsics,
                                                 const cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                                                 OccurenceMap & out_occurences,
                                                 const std::string& imagePath) const
{
  std::size_t frameCounter = 0;
  // for all the past frames
  for(const auto& frame : _frameBuffer)
  {
    // gather the data
    const auto &frameReconstructedRegions = frame._regionsWith3D;
    const auto &frameRegions = frame._regions;
    const auto &frameIntrinsics = frame._locResult.getIntrinsics();
    const auto frameImageSize = std::make_pair(frameIntrinsics.w(), frameIntrinsics.h());
    matching::MatchesPerDescType featureMatches;
    
    // match the query image with the current frame
    bool matchWorked = robustMatching(matchers,
                                      // pass the input intrinsic if they are valid, null otherwise
                                      (useInputIntrinsics) ? &queryIntrinsics : nullptr,
                                      frameRegions,
                                      &frameIntrinsics,
                                      param._fDistRatio,
                                      param._matchingError,
                                      param._useRobustMatching,
                                      param._useGuidedMatching,
                                      queryImageSize,
                                      frameImageSize, 
                                      featureMatches,
                                      param._matchingEstimator);
    if (!matchWorked)
    {
      continue;
    }

    OPENMVG_LOG_DEBUG("[matching]\tFound " << featureMatches.getNbAllMatches() << " matches from frame " << frameCounter);
    assert(featureMatches.getNbAllMatches() > 0);
    
    // recover the 2D-3D associations from the matches 
    // Each matched feature in the current similar image is associated to a 3D point

    std::size_t newOccurences = 0;
    for(const auto& featureMatchesIt : featureMatches)
    {
      features::EImageDescriberType descType = featureMatchesIt.first;
      for(const matching::IndMatch& featureMatch : featureMatchesIt.second)
      {
        // the ID of the 3D point
        const IndexT pt3D_id = frameReconstructedRegions.at(descType)._associated3dPoint.at(featureMatch._j);
        const IndexT pt2D_id = featureMatch._i;
        const OccurenceKey key(pt3D_id, descType, pt2D_id);

        if(out_occurences.count(key))
        {
          out_occurences[key]++;
        }
        else
        {
          OPENMVG_LOG_DEBUG("[matching]\tnew association found: [" << pt3D_id << "," << pt2D_id << "]");
          out_occurences[key] = 1;
          ++newOccurences;
        }
      }
    }
    OPENMVG_LOG_DEBUG("[matching]\tFound " << newOccurences << " new associations with the frameBufferMatching");
    ++frameCounter;
  }
}

bool VoctreeLocalizer::robustMatching(matching::RegionsDatabaseMatcherPerDesc & matchers,
                                      const cameras::IntrinsicBase * queryIntrinsicsBase,   // the intrinsics of the image we are using as reference
                                      const features::MapRegionsPerDesc & matchedRegions,
                                      const cameras::IntrinsicBase * matchedIntrinsicsBase,
                                      const float fDistRatio,
                                      const double matchingError,
                                      const bool useGeometricFiltering,
                                      const bool useGuidedMatching,
                                      const std::pair<std::size_t,std::size_t> & imageSizeI,     // size of the first image @fixme change the API of the kernel!! 
                                      const std::pair<std::size_t,std::size_t> & imageSizeJ,     // size of the second image
                                      matching::MatchesPerDescType & out_featureMatches,
                                      robust::EROBUST_ESTIMATOR estimator) const
{
  // get the intrinsics of the query camera
  if ((queryIntrinsicsBase != nullptr) && !isPinhole(queryIntrinsicsBase->getType()))
  {
    //@fixme maybe better to throw something here
    OPENMVG_CERR("[matching]\tOnly Pinhole cameras are supported!");
    return false;
  }
  const cameras::Pinhole_Intrinsic *queryIntrinsics = (const cameras::Pinhole_Intrinsic*)(queryIntrinsicsBase);
  
  // get the intrinsics of the matched view
  if ((matchedIntrinsicsBase != nullptr) &&  !isPinhole(matchedIntrinsicsBase->getType()) )
  {
    //@fixme maybe better to throw something here
    OPENMVG_CERR("[matching]\tOnly Pinhole cameras are supported!");
    return false;
  }
  const cameras::Pinhole_Intrinsic *matchedIntrinsics = (const cameras::Pinhole_Intrinsic*)(matchedIntrinsicsBase);
  
  const bool canBeUndistorted = (queryIntrinsicsBase != nullptr) && (matchedIntrinsicsBase != nullptr);

  // A. Putative Features Matching
  matching::MatchesPerDescType putativeFeatureMatches;
  const bool matchWorked = matchers.Match(fDistRatio, matchedRegions, putativeFeatureMatches);
  if (!matchWorked)
  {
    OPENMVG_LOG_DEBUG("[matching]\tPutative matching failed.");
    return false;
  }
  assert(!putativeFeatureMatches.empty());
  
  if(!useGeometricFiltering)
  {
    // nothing else to do
    out_featureMatches.swap(putativeFeatureMatches);
    return true;
  }

  // perform the geometric filtering
  matching_image_collection::GeometricFilter_FMatrix geometricFilter(matchingError, 5000, estimator);

  matching::MatchesPerDescType geometricInliersPerType;
  EstimationStatus estimationState = geometricFilter.geometricEstimation(
        matchers.getDatabaseRegionsPerDesc(),
        matchedRegions,
        queryIntrinsics,
        matchedIntrinsics,
        imageSizeI,
        imageSizeJ,
        putativeFeatureMatches,
        geometricInliersPerType);

  if(!estimationState.isValid)
  {
    OPENMVG_LOG_DEBUG("[matching]\tGeometric validation failed.");
    return false;
  }

  if(!estimationState.hasStrongSupport)
  {
    OPENMVG_LOG_DEBUG("[matching]\tGeometric validation hasn't strong support.");
    return false;
  }

  if(!useGuidedMatching)
  {
    out_featureMatches.swap(geometricInliersPerType);
    return true;
  }

  // Use the Fundamental Matrix estimated by the robust estimation to
  // perform guided matching.
  // So we ignore the previous matches and recompute all matches.
  out_featureMatches.clear();
  geometry_aware::GuidedMatching<
          Mat3,
          openMVG::fundamental::kernel::EpipolarDistanceError>(
        geometricFilter.m_F,
        queryIntrinsicsBase, // cameras::IntrinsicBase of the matched image
        matchers.getDatabaseRegionsPerDesc(), // features::Regions
        matchedIntrinsicsBase, // cameras::IntrinsicBase of the query image
        matchedRegions, // features::Regions
        Square(geometricFilter.m_dPrecision_robust),
        Square(fDistRatio),
        out_featureMatches); // output

  return true;
}

bool VoctreeLocalizer::localizeRig(const std::vector<image::Image<unsigned char> > & vec_imageGrey,
                                   const LocalizerParameters *parameters,
                                   std::vector<cameras::Pinhole_Intrinsic_Radial_K3 > &vec_queryIntrinsics,
                                   const std::vector<geometry::Pose3 > &vec_subPoses,
                                   geometry::Pose3 &rigPose, 
                                   std::vector<LocalizationResult> & vec_locResults)
{
  const size_t numCams = vec_imageGrey.size();
  assert(numCams == vec_queryIntrinsics.size());
  assert(numCams == vec_subPoses.size() + 1);

  std::vector<features::MapRegionsPerDesc> vec_queryRegions(numCams);
  std::vector<std::pair<std::size_t, std::size_t> > vec_imageSize;
  
  //@todo parallelize?
  for(size_t i = 0; i < numCams; ++i)
  {
    // add the image size for this image
    vec_imageSize.emplace_back(vec_imageGrey[i].Width(), vec_imageGrey[i].Height());

    // extract descriptors and features from each image
    for(auto& imageDescriber: _imageDescribers)
    {
      OPENMVG_LOG_DEBUG("[features]\tExtract " << features::EImageDescriberType_enumToString(imageDescriber->getDescriberType()) << " from query image...");
      imageDescriber->Describe(vec_imageGrey[i], vec_queryRegions[i][imageDescriber->getDescriberType()]);
      OPENMVG_LOG_DEBUG("[features]\tExtract done: found " <<  vec_queryRegions[i][imageDescriber->getDescriberType()]->RegionCount() << " features");
    }
    OPENMVG_LOG_DEBUG("[features]\tAll descriptors extracted. Found " <<  vec_queryRegions[i].getNbAllRegions() << " features");
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


bool VoctreeLocalizer::localizeRig(const std::vector<features::MapRegionsPerDesc> & vec_queryRegions,
                                   const std::vector<std::pair<std::size_t, std::size_t> > &vec_imageSize,
                                   const LocalizerParameters *parameters,
                                   std::vector<cameras::Pinhole_Intrinsic_Radial_K3 > &vec_queryIntrinsics,
                                   const std::vector<geometry::Pose3 > &vec_subPoses,
                                   geometry::Pose3 &rigPose,
                                   std::vector<LocalizationResult>& vec_locResults)
{
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_OPENGV)
  if(!parameters->_useLocalizeRigNaive)
  {
    OPENMVG_LOG_DEBUG("Using localizeRig_opengv()");
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


#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_OPENGV)

bool VoctreeLocalizer::localizeRig_opengv(const std::vector<features::MapRegionsPerDesc> & vec_queryRegions,
                                          const std::vector<std::pair<std::size_t, std::size_t> > &vec_imageSize,
                                          const LocalizerParameters *parameters,
                                          std::vector<cameras::Pinhole_Intrinsic_Radial_K3 > &vec_queryIntrinsics,
                                          const std::vector<geometry::Pose3 > &vec_subPoses,
                                          geometry::Pose3 &rigPose,
                                          std::vector<LocalizationResult>& vec_locResults)
{
  const std::size_t numCams = vec_queryRegions.size();
  assert(numCams == vec_queryIntrinsics.size());
  assert(numCams == vec_subPoses.size() + 1);   
  
  vec_locResults.clear();
  vec_locResults.reserve(numCams);
  
  const VoctreeLocalizer::Parameters *param = static_cast<const VoctreeLocalizer::Parameters *>(parameters);
  if(!param)
  {
    // error!
    throw std::invalid_argument("The parameters are not in the right format!!");
  }

  std::vector<OccurenceMap> vec_occurrences(numCams);
  std::vector<std::vector<voctree::DocMatch> > vec_matchedImages(numCams);
  std::vector< std::vector<features::EImageDescriberType> > descTypesPerCamera(numCams);
  std::vector<Mat> vec_pts3D(numCams);
  std::vector<Mat> vec_pts2D(numCams);

  // for each camera retrieve the associations
  //@todo parallelize?
  size_t numAssociations = 0;
  for(std::size_t camID = 0; camID < numCams; ++camID)
  {

    // this map is used to collect the 2d-3d associations as we go through the images
    // the key is a pair <Id3D, Id2d>
    // the element is the pair 3D point - 2D point
    auto &occurrences = vec_occurrences[camID];
    auto &matchedImages = vec_matchedImages[camID];
    auto &descTypes = descTypesPerCamera[camID];
    auto &imageSize = vec_imageSize[camID];
    Mat &pts3D = vec_pts3D[camID];
    Mat &pts2D = vec_pts2D[camID];
    cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics = vec_queryIntrinsics[camID];
    const bool useInputIntrinsics = true;
    getAllAssociations(vec_queryRegions[camID],
                       imageSize,
                       *param,
                       useInputIntrinsics,
                       queryIntrinsics,
                       occurrences,
                       pts2D,
                       pts3D,
                       descTypes,
                       matchedImages);
    numAssociations += occurrences.size();
  }
  
  // @todo Here it could be possible to filter the associations according to their
  // occurrences, eg giving priority to those associations that are more frequent

  const std::size_t minNumAssociations = 5;  //possible parameter?
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
  
  {
    for(std::size_t camID = 0; camID < vec_subPoses.size(); ++camID)
    {
    OPENMVG_LOG_DEBUG("Rotation: " << vec_subPoses[camID].rotation());
    OPENMVG_LOG_DEBUG("Centre: " << vec_subPoses[camID].center());
    }
  }
  
  std::vector<std::vector<std::size_t> > vec_inliers;
  const EstimationStatus resectionEstimation = rigResection(vec_pts2D,
                                        vec_pts3D,
                                        vec_queryIntrinsics,
                                        vec_subPoses,
                                        &descTypesPerCamera,
                                        rigPose,
                                        vec_inliers,
                                        param->_angularThreshold);

  if(!resectionEstimation.isValid)
  {
    for(std::size_t camID = 0; camID < numCams; ++camID)
    {
      // empty result with isValid set to false
      vec_locResults.emplace_back();
    }
    OPENMVG_LOG_DEBUG("Resection failed.");
    return false;
  }
  
  if(!resectionEstimation.hasStrongSupport)
  {
    OPENMVG_LOG_DEBUG("Resection hasn't a strong support.");
  }

  { // just debugging stuff, this block can be removed
    
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
    
    // print the reprojection error for inliers (just debugging purposes)
    printRigRMSEStats(vec_pts2D, vec_pts3D, vec_queryIntrinsics, vec_subPoses, rigPose, vec_inliers);
  }

//  const bool refineOk = refineRigPose(vec_pts2D,
//                                      vec_pts3D,
//                                      vec_inliers,
//                                      vec_queryIntrinsics,
//                                      vec_subPoses,
//                                      rigPose);
  
  const auto& resInl = computeInliers(vec_pts2D,
                                      vec_pts3D,
                                      vec_queryIntrinsics,
                                      vec_subPoses,
                                      param->_errorMax,
                                      rigPose,
                                      vec_inliers);
  
  OPENMVG_LOG_DEBUG("After first recomputation of inliers with a threshold of "
          << param->_errorMax << " the RMSE is: " << resInl.first);
  
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
    printRigRMSEStats(vec_pts2D, vec_pts3D, vec_queryIntrinsics, vec_subPoses, rigPose, vec_inliers);
  }
  
  // create localization results
  for(std::size_t camID = 0; camID < numCams; ++camID)
  {

    const auto &intrinsics = vec_queryIntrinsics[camID];

    // compute the (absolute) pose of each camera: for the main camera it's the 
    // rig pose, for the others, combine the subpose with the rig pose
    geometry::Pose3 pose;
    if(camID == 0)
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
      pose = vec_subPoses[camID-1] * rigPose;
    }
    
    // create matchData
    sfm::Image_Localizer_Match_Data matchData;
    matchData.vec_inliers = vec_inliers[camID];
    matchData.error_max = param->_errorMax;
    matchData.projection_matrix = intrinsics.get_projective_equivalent(pose);
    matchData.pt2D = vec_pts2D[camID];
    matchData.pt3D = vec_pts3D[camID];
    
    // create indMatch3D2D
    std::vector<IndMatch3D2D > indMatch3D2D;
    indMatch3D2D.reserve(matchData.pt2D.cols());
    const auto &occurrences = vec_occurrences[camID];
    for(const auto &ass : occurrences)
    {
      // recopy the associations IDs in the vector
      indMatch3D2D.push_back(ass.first);
    }
    
    const auto &matchedImages = vec_matchedImages[camID];
    
    vec_locResults.emplace_back(matchData, indMatch3D2D, pose, intrinsics, matchedImages, refineOk);
  }
  
  
  if(!refineOk)
  {
    OPENMVG_LOG_DEBUG("[poseEstimation]\tRefine failed.");
    return false;
  }
  
  { // just debugging stuff, this block can be removed
    
    OPENMVG_LOG_DEBUG("Pose after BA:");
    OPENMVG_LOG_DEBUG("Rotation: " << rigPose.rotation());
    OPENMVG_LOG_DEBUG("Centre: " << rigPose.center());
    
    // compute the reprojection error for inliers (just debugging purposes)
    for(std::size_t camID = 0; camID < numCams; ++camID)
    {
      const cameras::Pinhole_Intrinsic_Radial_K3 &currCamera = vec_queryIntrinsics[camID];
      Mat2X residuals;
      if(camID!=0)
        residuals = currCamera.residuals(vec_subPoses[camID-1]*rigPose, vec_pts3D[camID], vec_pts2D[camID]);
      else
        residuals = currCamera.residuals(geometry::Pose3()*rigPose, vec_pts3D[camID], vec_pts2D[camID]);

      const Vec sqrErrors = (residuals.cwiseProduct(residuals)).colwise().sum();

//      OPENMVG_LOG_DEBUG("Camera " << camID << " all reprojection errors after BA:");
//      OPENMVG_LOG_DEBUG(sqrErrors);

//      OPENMVG_LOG_DEBUG("Camera " << camID << " inliers reprojection errors after BA:");
      const auto &currInliers = vec_inliers[camID];

      double rmse = 0;
      for(std::size_t j = 0; j < currInliers.size(); ++j)
      {
//          OPENMVG_LOG_DEBUG(sqrErrors(currInliers[j]));
          rmse += sqrErrors(currInliers[j]);
      }
      if(!currInliers.empty())
        OPENMVG_LOG_DEBUG("\n\nRMSE inliers cam " << camID << ": " << std::sqrt(rmse/currInliers.size()));
    }
  }
    
  return true;
}

#endif // OPENMVG_HAVE_OPENGV

// subposes is n-1 as we consider the first camera as the main camera and the 
// reference frame of the grid
bool VoctreeLocalizer::localizeRig_naive(const std::vector<features::MapRegionsPerDesc> & vec_queryRegions,
                                          const std::vector<std::pair<std::size_t, std::size_t> > &vec_imageSize,
                                          const LocalizerParameters *parameters,
                                          std::vector<cameras::Pinhole_Intrinsic_Radial_K3 > &vec_queryIntrinsics,
                                          const std::vector<geometry::Pose3 > &vec_subPoses,
                                          geometry::Pose3 &rigPose,
                                          std::vector<LocalizationResult>& vec_localizationResults)
{
  const size_t numCams = vec_queryRegions.size();
  
  assert(numCams==vec_queryIntrinsics.size());
  assert(numCams==vec_subPoses.size()+1);
  assert(numCams==vec_imageSize.size());

  vec_localizationResults.resize(numCams);
    
  // this is basic, just localize each camera alone
  //@todo parallelize?
  std::vector<bool> isLocalized(numCams, false);
  for(size_t i = 0; i < numCams; ++i)
  {
    isLocalized[i] = localize(vec_queryRegions[i], vec_imageSize[i], parameters, true /*useInputIntrinsics*/, vec_queryIntrinsics[i], vec_localizationResults[i]);
    assert(isLocalized[i] == vec_localizationResults[i].isValid());
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



} // localization
} // openMVG
