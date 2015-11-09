#include "localization.hpp"


#include <openMVG/sfm/sfm_data_io.hpp>
#include <openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp>
#include <openMVG/features/io_regions_type.hpp>
#include <openMVG/matching/regions_matcher.hpp>
#include <openMVG/matching_image_collection/Matcher.hpp>
#include <openMVG/matching/matcher_kdtree_flann.hpp>
#include <openMVG/numeric/numeric.h>
#include <openMVG/robust_estimation/guided_matching.hpp>
#include <openMVG/matching_image_collection/F_ACRobust.hpp>
#include <third_party/progress/progress.hpp>
//#include <cereal/archives/json.hpp>

#include <algorithm>
#include <chrono>

//@fixme move/redefine
#define POPART_COUT(x) std::cout << x << std::endl
#define POPART_CERR(x) std::cerr << x << std::endl

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
  POPART_COUT("operator>>");
  int i;
  in >> i;
  a = static_cast<VoctreeLocalizer::Algorithm> (i);
  return in;
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

bool VoctreeLocalizer::localize( const image::Image<unsigned char> & imageGray,
                cameras::Pinhole_Intrinsic &queryIntrinsics,
                size_t numResults,
                geometry::Pose3 & pose,
                bool useGuidedMatching,
                bool useInputIntrinsics,
                bool refineIntrinsics,
                Algorithm algorithm,
                sfm::Image_Localizer_Match_Data &resection_data)
{
  switch(algorithm)
  {
    case Algorithm::FirstBest: return localizeFirstBestResult(imageGray, queryIntrinsics, numResults, pose, useGuidedMatching, useInputIntrinsics, refineIntrinsics, resection_data);
    case Algorithm::BestResult: throw std::invalid_argument("BestResult not yet implemented");
    case Algorithm::AllResults: return localizeAllResults(imageGray, queryIntrinsics, numResults, pose, useGuidedMatching, useInputIntrinsics, refineIntrinsics, resection_data);
    case Algorithm::Cluster: throw std::invalid_argument("Cluster not yet implemented");
    default: throw std::invalid_argument("Unknown algorithm type");
  }
}


// inputs
// - sfmdata path
// - descriptorsFolder directory with the sift
// - vocTreeFilepath; 
// - weightsFilepath; 
bool VoctreeLocalizer::init( const std::string &sfmFilePath,
                            const std::string &descriptorsFolder,
                            const std::string &vocTreeFilepath,
                            const std::string &weightsFilepath)
{
  using namespace openMVG::features;
  
  // load the sfm data containing the 3D reconstruction info
  POPART_COUT("Loading SFM data...");
  if (!Load(_sfm_data, sfmFilePath, sfm::ESfM_Data::ALL)) 
  {
    POPART_CERR("The input SfM_Data file "<< sfmFilePath << " cannot be read!");
    return false;
  }
  else
  {
    POPART_COUT("SfM data loaded from " << sfmFilePath << " containing: ");
    POPART_COUT("\tnumber of views      : " << _sfm_data.GetViews().size());
    POPART_COUT("\tnumber of poses      : " << _sfm_data.GetPoses().size());
    POPART_COUT("\tnumber of points     : " << _sfm_data.GetLandmarks().size());
    POPART_COUT("\tnumber of intrinsics : " << _sfm_data.GetIntrinsics().size());
  }

  // load the features and descriptors
  // initially we need all the feature in order to create the database
  // then we can store only those associated to 3D points
  //? can we use Feature_Provider to load the features and filter them later?
    
  initDatabase(vocTreeFilepath, weightsFilepath, descriptorsFolder);
  
  return true;
}


//@fixme deprecated.. now inside initDatabase
bool VoctreeLocalizer::loadReconstructionDescriptors(const sfm::SfM_Data & sfm_data,
                                                     const std::string & feat_directory)
{
  C_Progress_display my_progress_bar(sfm_data.GetViews().size(),
                                     std::cout, "\n- Regions Loading -\n");

  POPART_COUT("Build observations per view");
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

  POPART_COUT("Load Features and Descriptors per view");
  // Read for each view the corresponding regions and store them
  for(sfm::Views::const_iterator iter = sfm_data.GetViews().begin();
          iter != sfm_data.GetViews().end(); ++iter, ++my_progress_bar)
  {
    const IndexT id_view = iter->second->id_view;
    Reconstructed_RegionsT& reconstructedRegion = _regions_per_view[id_view];

    const std::string sImageName = stlplus::create_filespec(sfm_data.s_root_path, iter->second.get()->s_Img_path);
    const std::string basename = stlplus::basename_part(sImageName);
    const std::string featFilepath = stlplus::create_filespec(feat_directory, basename, ".feat");
    const std::string descFilepath = stlplus::create_filespec(feat_directory, basename, ".desc");
    //    std::cout << "Feat: " << featFilepath << std::endl;
    //    std::cout << "Desc: " << descFilepath << std::endl;

    if(!reconstructedRegion._regions.Load(featFilepath, descFilepath))
    {
      POPART_CERR("Invalid regions files for the view: " << sImageName);
      return false;
    }

    // Filter descriptors to keep only the 3D reconstructed points
    reconstructedRegion.filterRegions(observationsPerView[id_view]);
  }
  return true;
}

/**
 * @brief Initialize the database...
 */
bool VoctreeLocalizer::initDatabase(const std::string & vocTreeFilepath,
                                    const std::string & weightsFilepath,
                                    const std::string & feat_directory)
{

  bool withWeights = !weightsFilepath.empty();

  // Load vocabulary tree
  POPART_COUT("Loading vocabulary tree...");

  _voctree.load(vocTreeFilepath);
  POPART_COUT("tree loaded with" << endl << "\t" << _voctree.levels() << " levels" 
          << endl << "\t" << _voctree.splits() << " branching factor");

  POPART_COUT("Creating the database...");
  // Add each object (document) to the database
  _database = voctree::Database(_voctree.words());
  if(withWeights)
  {
    POPART_COUT("Loading weights...");
    _database.loadWeights(weightsFilepath);
  }
  else
  {
    POPART_COUT("No weights specified, skipping...");
  }
  
  // Load the descriptors and the features related to the images
  // for every image, pass the descriptors through the vocabulary tree and
  // add its visual words to the database.
  // then only store the feature and descriptors that have a 3D point associated
  POPART_COUT("Build observations per view");
  C_Progress_display my_progress_bar(_sfm_data.GetViews().size(),
                                     std::cout, "\n- Load Features and Descriptors per view -\n");

  // Build observations per view
  std::map<IndexT, std::vector<FeatureInImage> > observationsPerView;
  for(auto landmarkValue : _sfm_data.structure)
  {
    IndexT trackId = landmarkValue.first;
    sfm::Landmark& landmark = landmarkValue.second;
    for(auto obs : landmark.obs)
    {
      const IndexT viewId = obs.first;
      const sfm::Observation& obs2d = obs.second;
      observationsPerView[viewId].emplace_back(obs2d.id_feat, trackId);
    }
  }
  for(auto featuresInImage : observationsPerView)
  {
    std::sort(featuresInImage.second.begin(), featuresInImage.second.end());
  }

  // Read for each view the corresponding regions and store them
  for(const auto &iter : _sfm_data.GetViews())
  {
    const std::shared_ptr<sfm::View> currView = iter.second;
    const IndexT id_view = currView->id_view;
    Reconstructed_RegionsT& currRecoRegions = _regions_per_view[id_view];

    const std::string sImageName = stlplus::create_filespec(_sfm_data.s_root_path, currView.get()->s_Img_path);
    const std::string basename = stlplus::basename_part(sImageName);
    const std::string featFilepath = stlplus::create_filespec(feat_directory, basename, ".feat");
    const std::string descFilepath = stlplus::create_filespec(feat_directory, basename, ".desc");

    if(!currRecoRegions._regions.Load(featFilepath, descFilepath))
    {
      std::cerr << "Invalid regions files for the view: " << sImageName << std::endl;
      return false;
    }
    
    std::vector<voctree::Word> words = _voctree.quantize(currRecoRegions._regions.Descriptors());
    _database.insert(id_view, words);

    // Filter descriptors to keep only the 3D reconstructed points
    currRecoRegions.filterRegions(observationsPerView[id_view]);
    ++my_progress_bar;
  }
  
  return true;
}




//@todo move the parameters into a struct
bool VoctreeLocalizer::localizeFirstBestResult(const image::Image<unsigned char> & imageGray,
                                cameras::Pinhole_Intrinsic &queryIntrinsics,
                                size_t numResults,
                                geometry::Pose3 & pose,
                                bool useGuidedMatching,
                                bool useInputIntrinsics,
                                bool refineIntrinsics,
                                sfm::Image_Localizer_Match_Data &resectionData)
{
  // A. extract descriptors and features from image
  POPART_COUT("[features]\tExtract SIFT from query image");
  std::unique_ptr<features::Regions> tmpQueryRegions(new features::SIFT_Float_Regions());
  auto detect_start = std::chrono::steady_clock::now();
  _image_describer.Describe(imageGray, tmpQueryRegions, nullptr);
  auto detect_end = std::chrono::steady_clock::now();
  auto detect_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(detect_end - detect_start);
  POPART_COUT("[features]\tExtract SIFT done: found " << tmpQueryRegions->RegionCount() << " features in " << detect_elapsed.count() << " [ms]" );
  features::SIFT_Float_Regions queryRegions = *dynamic_cast<features::SIFT_Float_Regions*> (tmpQueryRegions.get());

  // B. Find the (visually) similar images in the database 
  POPART_COUT("[database]\tRequest closest images from voctree");
  // pass the descriptors through the vocabulary tree to get the visual words
  // associated to each feature
  std::vector<voctree::Word> requestImageWords = _voctree.quantize(queryRegions.Descriptors());
  
  // Request closest images from voctree
  std::vector<voctree::Match> matchedImages;
  _database.find(requestImageWords, numResults, matchedImages);
  
  // just debugging bla bla
  // for each similar image found print score and number of features
  for(const voctree::Match & currMatch : matchedImages )
  {
    // get the corresponding index of the view
    const IndexT matchedViewIndex = currMatch.id;
    // get the view handle
    const std::shared_ptr<sfm::View> matchedView = _sfm_data.views[matchedViewIndex];
    POPART_COUT( "[database]\t\t match " << matchedView->s_Img_path 
            << " [docid: "<< currMatch.id << "]"
            << " with score " << currMatch.score 
            << " and it has "  << _regions_per_view[matchedViewIndex]._regions.RegionCount() 
            << " features with 3D points");
  }

  //@fixme Maybe useless, just do everything with DistanceRatioMatch
  // preparing the matcher, it will use the extracted Regions as reference and it
  // will match them to the Regions of each similar image
  const float fDistRatio = 0.6; //@fixme this could be a param
//  typedef flann::L2<unsigned char> MetricT;
//  typedef matching::ArrayMatcher_Kdtree_Flann<unsigned char, MetricT> MatcherT;
  POPART_COUT("[matching]\tBuilding the matcher");
  matching::RegionsMatcherT<MatcherT> matcher(queryRegions);
  
 
  // C. for each found similar image, try to find the correspondences between the 
  // query image and the similar image
  for(const voctree::Match& matchedImage : matchedImages)
  {
    // minimum number of points that allows a reliable 3D reconstruction
    const size_t minNum3DPoints = 5;
    
    // the view index of the current matched image
    const IndexT matchedViewIndex = matchedImage.id;
    // the handler to the current view
    const std::shared_ptr<sfm::View> matchedView = _sfm_data.views[matchedViewIndex];
    
    // safeguard: we should match the query image with an image that has at least
    // some 3D points visible --> if it has 0 3d points it is likely that it is an
    // image of the dataset that was not reconstructed
    if(_regions_per_view[matchedViewIndex]._regions.RegionCount() < minNum3DPoints)
    {
      POPART_COUT("[matching]\tSkipping matching with " << matchedView->s_Img_path << " as it has too few visible 3D points");
      continue;
    }
    else
    {
      POPART_COUT("[matching]\tTrying to match the query image with " << matchedView->s_Img_path);
    }
    
    // its associated reconstructed regions
    const Reconstructed_RegionsT& matchedRegions = _regions_per_view[matchedViewIndex];
    // its associated intrinsics
    // this is just ugly!
    const cameras::IntrinsicBase *matchedIntrinsicsBase = _sfm_data.intrinsics[matchedView->id_intrinsic].get();
    if ( !isPinhole(matchedIntrinsicsBase->getType()) )
    {
      //@fixme maybe better to throw something here
      POPART_CERR("Only Pinhole cameras are supported!");
      return false;
    }
    const cameras::Pinhole_Intrinsic *matchedIntrinsics = (const cameras::Pinhole_Intrinsic*)(matchedIntrinsicsBase);
     
    std::vector<matching::IndMatch> vec_featureMatches;
    bool matchWorked = robustMatching( matcher, 
                                      // pass the input intrinsic if they are valid, null otherwise
                                      (useInputIntrinsics) ? &queryIntrinsics : nullptr,
                                      matchedRegions,
                                      matchedIntrinsics,
                                      fDistRatio,
                                      useGuidedMatching,
                                      std::make_pair(imageGray.Width(), imageGray.Height()),
                                      std::make_pair(imageGray.Width(), imageGray.Height()), // NO! @fixme here we need the size of the img in the dataset...
                                      vec_featureMatches);
    if (!matchWorked)
    {
      POPART_COUT("[matching]\tMatching with " << matchedView->s_Img_path << " failed! Skipping image");
      continue;
    }
    else
    {
      POPART_COUT("[matching]\tFound " << vec_featureMatches.size() << " matches");
    }
    assert(vec_featureMatches.size()>0);
    
    // D. recover the 2D-3D associations from the matches 
    // Each matched feature in the current similar image is associated to a 3D point,
    // hence we can recover the 2D-3D associations to estimate the pose
    // Prepare data for resection
    resectionData = sfm::Image_Localizer_Match_Data();
    resectionData.pt2D = Mat2X(2, vec_featureMatches.size());
    resectionData.pt3D = Mat3X(3, vec_featureMatches.size());

    // Get the 3D points associated to each matched feature
    std::size_t index = 0;
    for(const matching::IndMatch& featureMatch : vec_featureMatches)
    {
      assert(vec_featureMatches.size()>index);
      // the ID of the 3D point
      const IndexT trackId3D = matchedRegions._associated3dPoint[featureMatch._j];

      // prepare data for resectioning
      resectionData.pt3D.col(index) = _sfm_data.GetLandmarks().at(trackId3D).X;

      const Vec2 feat = queryRegions.GetRegionPosition(featureMatch._i);
      // if the intrinsics are known undistort the points
      if(useInputIntrinsics)
      {
        resectionData.pt2D.col(index) = queryIntrinsics.get_ud_pixel(feat);
      }
      else
      {
        resectionData.pt2D.col(index) = feat;
      }

      ++index;
    }
    // estimate the pose
    // Do the resectioning: compute the camera pose.
    resectionData.error_max = std::numeric_limits<double>::max();
    POPART_COUT("[poseEstimation]\tEstimating camera pose...");
    bool bResection = sfm::SfM_Localizer::Localize(std::make_pair(imageGray.Width(), imageGray.Height()),
                                                   // pass the input intrinsic if they are valid, null otherwise
                                                   (useInputIntrinsics) ? &queryIntrinsics : nullptr,
                                                   resectionData,
                                                   pose);

    if(!bResection)
    {
      POPART_COUT("[poseEstimation]\tResection FAILED");
      continue;
    }
    POPART_COUT("[poseEstimation]\tResection SUCCEDED");
       
    POPART_COUT("R est\n" << pose.rotation());
    POPART_COUT("t est\n" << pose.translation());

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
      POPART_COUT("K estimated\n" << K_);
      queryIntrinsics.setK(K_);
      queryIntrinsics.setWidth(imageGray.Width());
      queryIntrinsics.setHeight(imageGray.Height());
    }

    // E. refine the estimated pose
    POPART_COUT("[poseEstimation]\tRefining estimated pose");
    bool refineStatus = sfm::SfM_Localizer::RefinePose(&queryIntrinsics, 
                                                       pose, 
                                                       resectionData, 
                                                       true /*b_refine_pose*/, 
                                                       refineIntrinsics /*b_refine_intrinsic*/);
    if(!refineStatus)
      POPART_COUT("[poseEstimation]\tRefine pose could not improve the estimation of the camera pose.");
    
    {
      // just temporary code to evaluate the estimated pose @todo remove it
      const geometry::Pose3 &referencePose = _sfm_data.poses[matchedViewIndex];
      POPART_COUT("R refined\n" << pose.rotation());
      POPART_COUT("t refined\n" << pose.translation());
      POPART_COUT("K refined\n" << queryIntrinsics.K());
      POPART_COUT("R_gt\n" << referencePose.rotation());
      POPART_COUT("t_gt\n" << referencePose.translation());
      POPART_COUT("angular difference: " << R2D(getRotationMagnitude(pose.rotation()*referencePose.rotation().inverse())) << "deg");
      POPART_COUT("center difference: " << (pose.center()-referencePose.center()).norm());
      POPART_COUT("err = [err; " << R2D(getRotationMagnitude(pose.rotation()*referencePose.rotation().inverse())) << ", "<< (pose.center()-referencePose.center()).norm() << "];");
    }
    break;
  }
  //@todo deal with unsuccesful case...
  return true;
  
 } 


bool VoctreeLocalizer::localizeAllResults(const image::Image<unsigned char> & imageGray,
                                          cameras::Pinhole_Intrinsic &queryIntrinsics,
                                          size_t numResults,
                                          geometry::Pose3 & pose,
                                          bool useGuidedMatching,
                                          bool useInputIntrinsics,
                                          bool refineIntrinsics,
                                          sfm::Image_Localizer_Match_Data &resectionData)
{
  // A. extract descriptors and features from image
  POPART_COUT("[features]\tExtract SIFT from query image");
  std::unique_ptr<features::Regions> tmpQueryRegions(new features::SIFT_Float_Regions());
  auto detect_start = std::chrono::steady_clock::now();
  _image_describer.Describe(imageGray, tmpQueryRegions, nullptr);
  auto detect_end = std::chrono::steady_clock::now();
  auto detect_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(detect_end - detect_start);
  POPART_COUT("[features]\tExtract SIFT done: found " << tmpQueryRegions->RegionCount() << " features in " << detect_elapsed.count() << " [ms]" );
  features::SIFT_Float_Regions queryRegions = *dynamic_cast<features::SIFT_Float_Regions*> (tmpQueryRegions.get());

  // B. Find the (visually) similar images in the database 
  // pass the descriptors through the vocabulary tree to get the visual words
  // associated to each feature
  POPART_COUT("[database]\tRequest closest images from voctree");
  std::vector<voctree::Word> requestImageWords = _voctree.quantize(queryRegions.Descriptors());
  
  // Request closest images from voctree
  std::vector<voctree::Match> matchedImages;
  if(numResults==0) numResults = _database.size();
  _database.find(requestImageWords, numResults, matchedImages);
  
  // just debugging bla bla
  // for each similar image found print score and number of features
  for(const voctree::Match & currMatch : matchedImages )
  {
    // get the view handle
    const std::shared_ptr<sfm::View> matchedView = _sfm_data.views[currMatch.id];
    POPART_COUT( "[database]\t\t match " << matchedView->s_Img_path 
            << " [docid: "<< currMatch.id << "]"
            << " with score " << currMatch.score 
            << " and it has "  << _regions_per_view[currMatch.id]._regions.RegionCount() 
            << " features with 3D points");
  }

  const float fDistRatio = 0.6; //@fixme this could be a param

  POPART_COUT("[matching]\tBuilding the matcher");
  matching::RegionsMatcherT<MatcherT> matcher(queryRegions);

  std::vector<Vec3> collected3Dpts;
  std::vector<Vec2> collected2Dpts;
  
  // C. for each found similar image, try to find the correspondences between the 
  // query image adn the similar image
  std::size_t maxResults = 10;
  for(const voctree::Match& matchedImage : matchedImages)
  {
    // minimum number of points that allows a reliable 3D reconstruction
    const size_t minNum3DPoints = 5;
    
    // the handler to the current view
    const std::shared_ptr<sfm::View> matchedView = _sfm_data.views[matchedImage.id];
    // its associated reconstructed regions
    const Reconstructed_RegionsT& matchedRegions = _regions_per_view[matchedImage.id];
    
    // safeguard: we should match the query image with an image that has at least
    // some 3D points visible --> if this is not true it is likely that it is an
    // image of the dataset that was not reconstructed
    if(matchedRegions._regions.RegionCount() < minNum3DPoints)
    {
      POPART_COUT("[matching]\tSkipping matching with " << matchedView->s_Img_path << " as it has too few visible 3D points");
      continue;
    }
    else
    {
      POPART_COUT("[matching]\tTrying to match the query image with " << matchedView->s_Img_path);
    }
    
    // its associated intrinsics
    // this is just ugly!
    const cameras::IntrinsicBase *matchedIntrinsicsBase = _sfm_data.intrinsics[matchedView->id_intrinsic].get();
    if ( !isPinhole(matchedIntrinsicsBase->getType()) )
    {
      //@fixme maybe better to throw something here
      POPART_CERR("Only Pinhole cameras are supported!");
      return false;
    }
    const cameras::Pinhole_Intrinsic *matchedIntrinsics = (const cameras::Pinhole_Intrinsic*)(matchedIntrinsicsBase);
     
    std::vector<matching::IndMatch> vec_featureMatches;
    bool matchWorked = robustMatching( matcher, 
                                      // pass the input intrinsic if they are valid, null otherwise
                                      (useInputIntrinsics) ? &queryIntrinsics : nullptr,
                                      matchedRegions,
                                      matchedIntrinsics,
                                      fDistRatio,
                                      useGuidedMatching,
                                      std::make_pair(imageGray.Width(), imageGray.Height()),
                                      std::make_pair(imageGray.Width(), imageGray.Height()), // NO! @fixme here we need the size of the img in the dataset...
                                      vec_featureMatches);
    if (!matchWorked)
    {
      POPART_COUT("[matching]\tMatching with " << matchedView->s_Img_path << " failed! Skipping image");
      continue;
    }
    else
    {
      POPART_COUT("[matching]\tFound " << vec_featureMatches.size() << " matches");
    }
    assert(vec_featureMatches.size()>0);
    
    // D. recover the 2D-3D associations from the matches 
    // Each matched feature in the current similar image is associated to a 3D point,
    // hence we can recover the 2D-3D associations to estimate the pose
    // Prepare data for resection
    
    std::size_t index = 0;
    for(const matching::IndMatch& featureMatch : vec_featureMatches)
    {
      assert(vec_featureMatches.size()>index);
      // the ID of the 3D point
      const IndexT trackId3D = matchedRegions._associated3dPoint[featureMatch._j];

      // Get the 3D point
      collected3Dpts.emplace_back(_sfm_data.GetLandmarks().at(trackId3D).X);

      // Get the 2D point
      const Vec2 feat = queryRegions.GetRegionPosition(featureMatch._i);
      // if the intrinsics are known undistort the points
      if(useInputIntrinsics)
      {
        collected2Dpts.emplace_back( queryIntrinsics.get_ud_pixel(feat) );
      }
      else
      {
        collected2Dpts.emplace_back(feat);
      }

      ++index;
    }
    --maxResults;
    if( maxResults<1 )
    { 
      // let's say we have enough features
      POPART_COUT("[matching]\tgot enough point from " << maxResults << "images");
      break;
    }
  }
  assert(collected3Dpts.size() == collected2Dpts.size());
  size_t numCollectedPts = collected3Dpts.size();
  
  resectionData = sfm::Image_Localizer_Match_Data();
  resectionData.pt2D = Mat2X(2, numCollectedPts);
  resectionData.pt3D = Mat3X(3, numCollectedPts);
  
  for(std::size_t index = 0; index < numCollectedPts; ++index)
  {
     resectionData.pt2D.col(index) = collected2Dpts[index];
     resectionData.pt3D.col(index) = collected3Dpts[index];
  }
  
  
  // estimate the pose
  // Do the resectioning: compute the camera pose.
  resectionData.error_max = std::numeric_limits<double>::max();
  POPART_COUT("[poseEstimation]\tEstimating camera pose...");
  bool bResection = sfm::SfM_Localizer::Localize(std::make_pair(imageGray.Width(), imageGray.Height()),
                                                 // pass the input intrinsic if they are valid, null otherwise
                                                 (useInputIntrinsics) ? &queryIntrinsics : nullptr,
                                                 resectionData,
                                                 pose);

  if(!bResection)
  {
    POPART_COUT("[poseEstimation]\tResection FAILED");
    return false;
  }
  POPART_COUT("[poseEstimation]\tResection SUCCEDED");

  POPART_COUT("R est\n" << pose.rotation());
  POPART_COUT("t est\n" << pose.translation());

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
    POPART_COUT("K estimated\n" <<K_);
    queryIntrinsics.setWidth(imageGray.Width());
    queryIntrinsics.setHeight(imageGray.Height());
  }

  // E. refine the estimated pose
  POPART_COUT("[poseEstimation]\tRefining estimated pose");
  bool refineStatus = sfm::SfM_Localizer::RefinePose(&queryIntrinsics, 
                                                     pose, 
                                                     resectionData, 
                                                     true /*b_refine_pose*/, 
                                                     refineIntrinsics /*b_refine_intrinsic*/);
  if(!refineStatus)
    POPART_COUT("Refine pose could not improve the estimation of the camera pose.");

  {
    // just temporary code to evaluate the estimated pose @todo remove it
    POPART_COUT("R refined\n" << pose.rotation());
    POPART_COUT("t refined\n" << pose.translation());
    POPART_COUT("K refined\n" << queryIntrinsics.K());
  }
  //@todo deal with unsuccesful case...
  return true;
}



bool VoctreeLocalizer::robustMatching(matching::RegionsMatcherT<MatcherT> & matcher, 
                    const cameras::IntrinsicBase * queryIntrinsics,   // the intrinsics of the image we are using as reference
                    const Reconstructed_RegionsT & matchedRegions,
                    const cameras::IntrinsicBase * matchedIntrinsics,
                    const float fDistRatio,
                    const bool b_guided_matching,
                    const std::pair<size_t,size_t> & imageSizeI,     // size of the first image @fixme change the API of the kernel!! 
                    const std::pair<size_t,size_t> & imageSizeJ,     // size of the first image
                    std::vector<matching::IndMatch> & vec_featureMatches) const
{
  // A. Putative Features Matching
  bool matchWorked = matcher.Match(fDistRatio, matchedRegions._regions, vec_featureMatches);
  if (!matchWorked)
  {
    POPART_COUT("\tRobust matching failed!");
    return false;
  }
  assert(vec_featureMatches.size()>0);
  // prepare the data for geometric filtering: for each matched pair of features,
  // store them in two matrices
  Mat featuresI(2, vec_featureMatches.size());
  Mat featuresJ(2, vec_featureMatches.size());
  // fill the matrices with the features according to vec_featureMatches
  for(int i = 0; i < vec_featureMatches.size(); ++i)
  {
    const matching::IndMatch& match = vec_featureMatches[i];
    featuresI.col(i) = matcher.getDatabaseRegions()->GetRegionPosition(match._i);
    featuresJ.col(i) = matchedRegions._regions.GetRegionPosition(match._j);
  }
  // perform the geometric filtering
  matching_image_collection::GeometricFilter_FMatrix_AC geometricFilter(4.0);
  std::vector<size_t> vec_matchingInliers;
  bool valid = geometricFilter.Robust_estimation(featuresI, // points of the query image
                                                 featuresJ, // points of the matched image
                                                 imageSizeI,
                                                 imageSizeJ,
                                                 vec_matchingInliers);
  if(!valid)
  {
    POPART_COUT("[matching]\tUnable to robustly matching the query image with the database image.");
    return false;
  }
  if(!b_guided_matching)
  {
    // prepare to output vec_featureMatches
    // the indices stored in vec_matchingInliers refer to featuresI, now we need
    // to recover the original indices wrt matchedRegions and queryRegions and fill 
    // a temporary vector.
    std::vector<matching::IndMatch> vec_robustFeatureMatches;
    vec_robustFeatureMatches.reserve(vec_matchingInliers.size());
    for(const size_t idx : vec_matchingInliers)
    {
      // use the index stored in vec_matchingInliers to get the indices of the 
      // original matching features and store them in vec_robustFeatureMatches
      vec_robustFeatureMatches.emplace_back(vec_featureMatches[idx]);
    }
    // just swap the vector so the output is ready
    std::swap(vec_robustFeatureMatches, vec_featureMatches);
  }
  else
  {
    // Use the Fundamental Matrix estimated by the robust estimation to
    // perform guided matching.
    // So we ignore the previous matches and recompute all matches.
    vec_featureMatches.clear();
    geometry_aware::GuidedMatching<
            Mat3,
            openMVG::fundamental::kernel::EpipolarDistanceError>(
            geometricFilter.m_F,
            queryIntrinsics, // cameras::IntrinsicBase of the matched image
            *matcher.getDatabaseRegions(), // features::Regions
            matchedIntrinsics, // cameras::IntrinsicBase of the query image
            matchedRegions._regions, // features::Regions
            Square(geometricFilter.m_dPrecision_robust),
            Square(fDistRatio),
            vec_featureMatches); // output
  }
  return true;
}

} // localization
} // openMVG
