#include "KDTreeLocalizer.hpp"
#include "rigResection.hpp"
#include "optimization.hpp"

#include <openMVG/sfm/sfm_data_io.hpp>
#include <openMVG/sfm/pipelines/sfm_robust_model_estimation.hpp>
#include <openMVG/sfm/sfm_data_BA_ceres.hpp>
#include <openMVG/features/io_regions_type.hpp>
#include <openMVG/features/svgVisualization.hpp>
#ifdef HAVE_CCTAG
#include <openMVG/features/cctag/SIFT_CCTAG_describer.hpp>
#endif
#include <openMVG/matching_image_collection/Matcher.hpp>
#include <openMVG/matching_image_collection/F_ACRobust.hpp>
#include <nonFree/sift/SIFT_float_describer.hpp>
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

KDTreeLocalizer::KDTreeLocalizer(const std::string &sfmFilePath, const std::string &descriptorsFolder, size_t leafSize
#ifdef HAVE_CCTAG
                                   , bool useSIFT_CCTAG
#endif
                                  ) : _leafSize(leafSize)
{
  using namespace openMVG::features;
  // init the feature extractor
#ifdef HAVE_CCTAG
  if(useSIFT_CCTAG)
  {
    OPENMVG_LOG_DEBUG("SIFT_CCTAG_Image_describer");
    _image_describer = new features::SIFT_CCTAG_Image_describer();  
  }
  else
  {
#if USE_SIFT_FLOAT
    OPENMVG_LOG_DEBUG("SIFT_float_describer");
    _image_describer = new features::SIFT_float_describer();
#else
    OPENMVG_LOG_DEBUG("SIFT_Image_describer");
    _image_describer = new features::SIFT_Image_describer();
#endif
  }
#else //HAVE_CCTAG
#if USE_SIFT_FLOAT
    OPENMVG_LOG_DEBUG("SIFT_float_describer");
    _image_describer = new features::SIFT_float_describer();
#else
    OPENMVG_LOG_DEBUG("SIFT_Image_describer");
    _image_describer = new features::SIFT_Image_describer();
#endif
#endif //HAVE_CCTAG
  
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

  _isInit = initDatabase(descriptorsFolder);
}

bool KDTreeLocalizer::localize(const std::unique_ptr<features::Regions> &genQueryRegions,
                                const std::pair<std::size_t, std::size_t> &imageSize,
                                const LocalizerParameters *param,
                                bool useInputIntrinsics,
                                cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                                LocalizationResult & localizationResult,
                                const std::string& imagePath)
{
    const Parameters *voctreeParam = static_cast<const Parameters *>(param);
    if (!voctreeParam)
    {
        // error!
        throw std::invalid_argument("The parameters are not in the right format!!");
    }

#if USE_SIFT_FLOAT  
    features::SIFT_Float_Regions *queryRegions = dynamic_cast<features::SIFT_Float_Regions*> (genQueryRegions.get());
#else
    features::SIFT_Regions *queryRegions = dynamic_cast<features::SIFT_Regions*> (genQueryRegions.get());
#endif

    if (!queryRegions)
    {
        // error, the casting did not work
        throw std::invalid_argument("Error while converting the input Regions. Only SIFT regions are allowed for the voctree localizer.");
    }

    return localizeAllResults(*queryRegions,
        imageSize,
        *voctreeParam,
        useInputIntrinsics,
        queryIntrinsics,
        localizationResult,
        imagePath);
}

bool KDTreeLocalizer::localize(const image::Image<unsigned char> & imageGrey,
                                const LocalizerParameters *param,
                                bool useInputIntrinsics,
                                cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                                LocalizationResult &localizationResult,
                                const std::string& imagePath /* = std::string() */)
{
  // A. extract descriptors and features from image
  OPENMVG_LOG_DEBUG("[features]\tExtract SIFT from query image");
#if USE_SIFT_FLOAT
  std::unique_ptr<features::Regions> tmpQueryRegions(new features::SIFT_Float_Regions());
#else
  std::unique_ptr<features::Regions> tmpQueryRegions(new features::SIFT_Regions());
#endif
  auto detect_start = std::chrono::steady_clock::now();
  _image_describer->Set_configuration_preset(param->_featurePreset);
  _image_describer->Describe(imageGrey, tmpQueryRegions, nullptr);
  auto detect_end = std::chrono::steady_clock::now();
  auto detect_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(detect_end - detect_start);
  OPENMVG_LOG_DEBUG("[features]\tExtract SIFT done: found " << tmpQueryRegions->RegionCount() << " features in " << detect_elapsed.count() << " [ms]");

  const std::pair<std::size_t, std::size_t> queryImageSize = std::make_pair(imageGrey.Width(), imageGrey.Height());

  // if debugging is enable save the svg image with the extracted features
  if(!param->_visualDebug.empty() && !imagePath.empty())
  {
    namespace bfs = boost::filesystem;
    saveFeatures2SVG(imagePath,
                     queryImageSize,
                     tmpQueryRegions->GetRegionsPositions(),
                     param->_visualDebug + "/" + bfs::path(imagePath).stem().string() + ".svg");
  }

  return localize(tmpQueryRegions,
                  queryImageSize,
                  param,
                  useInputIntrinsics,
                  queryIntrinsics,
                  localizationResult,
                  imagePath);
}

/**
 * @brief Initialize the database...
 */
bool KDTreeLocalizer::initDatabase(const std::string & feat_directory)
{
  // Load the descriptors and the features related to the images for every image.
  // then only store the feature and descriptors that have a 3D point associated
  OPENMVG_LOG_DEBUG("Build observations per view");
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

  std::vector<popsift::kdtree::U8Descriptor> globalDescriptorDB;
  std::vector<unsigned short> globalDescriptorAssoc;

  // Read for each view the corresponding Regions and store them
  for(const auto &iter : _sfm_data.GetViews())
  {
    const std::shared_ptr<sfm::View> currView = iter.second;
    const IndexT id_view = currView->id_view;
    Reconstructed_RegionsT& currRecoRegions = _regions_per_view[id_view];

    const std::string sImageName = stlplus::create_filespec(_sfm_data.s_root_path, currView->s_Img_path);
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

    if(!currRecoRegions._regions.Load(featFilepath, descFilepath))
    {
      OPENMVG_CERR("Invalid regions files for the view: " << sImageName);
      return false;
    }
    
    // Filter descriptors to keep only the 3D reconstructed points
    currRecoRegions.filterRegions(observationsPerView[id_view]);

    // Insert them into the global DB.  
    size_t addCount = currRecoRegions._regions.RegionCount();
    if (addCount < 5) {
        OPENMVG_LOG_DEBUG("Image not added to KD-tree; <5 visible points: " << currView->s_Img_path);
    }
    else {
        size_t oldCount = globalDescriptorDB.size();
        globalDescriptorDB.resize(oldCount + addCount);
        memcpy(&globalDescriptorDB[oldCount], currRecoRegions._regions.DescriptorRawData(), addCount * sizeof(popsift::kdtree::U8Descriptor));
        globalDescriptorAssoc.insert(globalDescriptorAssoc.end(), addCount, static_cast<unsigned short>(id_view));
    }

    ++my_progress_bar;
  }

  // Grow the forest.
  assert(globalDescriptorAssoc.size() == globalDescriptorDB.size());
  _kdtrees = popsift::kdtree::Build(globalDescriptorDB.data(), globalDescriptorAssoc.data(), globalDescriptorDB.size(), 10, _leafSize);
  return true;
}

bool KDTreeLocalizer::localizeAllResults(const features::SIFT_Regions &queryRegions,
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
  std::map< std::pair<IndexT, IndexT>, std::size_t > occurences;
  
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
                     matchedImages,
                     imagePath);

  const std::size_t numCollectedPts = occurences.size();
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

  return localizationResult.isValid();
}

struct Q2NNAccumulator
{
    unsigned distance[2];
    unsigned index[2];

    Q2NNAccumulator()
    {
        distance[0] = distance[1] = std::numeric_limits<unsigned>::max();
        index[0] = index[1] = std::numeric_limits<unsigned>::max();
    }

    void Update(unsigned d, unsigned i)
    {
        if (d < distance[0]) {
            distance[1] = distance[0]; distance[0] = d;
            index[1] = index[0]; index[0] = i;
        }
        else if (d != distance[0] && d < distance[1]) {
            distance[1] = d;
            index[1] = i;
        }
    }
};

void KDTreeLocalizer::getAllAssociations(const features::SIFT_Regions &queryRegions,
                                          const std::pair<std::size_t, std::size_t> &imageSize,
                                          const Parameters &param,
                                          bool useInputIntrinsics,
                                          const cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                                          std::map< std::pair<IndexT, IndexT>, std::size_t > &occurences,
                                          Mat &pt2D,
                                          Mat &pt3D,
                                          std::vector<voctree::DocMatch>& matchedImages,
                                          const std::string& imagePath) const
{
    using std::get;
    static constexpr size_t MAX_SEARCH_CANDIDATES = 1000;  // ~60% 1NN and ~33% 2NN accuracy on SIFT1M dataset

    // A. 2-ANN search in the global db for matches.
    auto descriptorMatches = popsift::kdtree::Query2NN(_kdtrees, MAX_SEARCH_CANDIDATES,
        static_cast<const popsift::kdtree::U8Descriptor*>(queryRegions.DescriptorRawData()),
        queryRegions.RegionCount());

    // Group matches by DB image id on 1-NN.
    std::sort(descriptorMatches.begin(), descriptorMatches.end(), [](
        const popsift::kdtree::QueryResult::value_type& a1,
        const popsift::kdtree::QueryResult::value_type& a2)
    {
        return get<1>(a1).image_index < get<1>(a2).image_index;
    });

    // B. for each found similar image, try to find the correspondences between the 
    // query image adn the similar image
    // stop when param._maxResults successful matches have been found
    std::size_t goodMatches = 0;
    popsift::kdtree::QueryResult::iterator
        itMatchingImageBegin = descriptorMatches.begin(), itMatchingImageEnd;

    // Iterate over groups of matches within the same image.
    for (; itMatchingImageBegin != descriptorMatches.end(); itMatchingImageBegin = itMatchingImageEnd) {
        // [itMatchingImageBegin,itMatchingImageEnd) are now all matches for the same DB image.
        const std::shared_ptr<sfm::View> matchedView = _sfm_data.views.at(get<1>(*itMatchingImageBegin).image_index);
        const Reconstructed_RegionsT& matchedRegions = _regions_per_view.at(get<1>(*itMatchingImageBegin).image_index);

        // Find the end of current range having the 1-NN in the same image.
        itMatchingImageEnd = std::find_if(itMatchingImageBegin, descriptorMatches.end(),
            [&](const popsift::kdtree::QueryResult::value_type& a) {
            return get<1>(a).image_index != get<1>(*itMatchingImageBegin).image_index;
        });

        // safeguard: we should match the query image with an image that has at least
        // some 3D points visible --> if this is not true it is likely that it is an
        // image of the dataset that was not reconstructed.  This should never happen
        // since we never insert descriptors from images having < 5 points.
        if (matchedRegions._regions.RegionCount() < 5)  // XXX: hard-coded! minimum number of points that allows a reliable 3D reconstruction
        {
            OPENMVG_LOG_DEBUG("[matching]\tSkipping matching with " << matchedView->s_Img_path << " as it has too few visible 3D points");
            continue;
        }
        OPENMVG_LOG_DEBUG("[matching]\tTrying to match the query image with " << matchedView->s_Img_path);
        OPENMVG_LOG_DEBUG("[matching]\tIt has " << matchedRegions._regions.RegionCount() << " available features to match");

        // its associated intrinsics; XXX: this is just ugly!
        const cameras::IntrinsicBase *matchedIntrinsicsBase = _sfm_data.intrinsics.at(matchedView->id_intrinsic).get();
        const cameras::Pinhole_Intrinsic *matchedIntrinsics = (const cameras::Pinhole_Intrinsic*)(matchedIntrinsicsBase);
        if (!isPinhole(matchedIntrinsicsBase->getType()))
        {
            //@fixme maybe better to throw something here
            OPENMVG_CERR("Only Pinhole cameras are supported!");
            return;
        }

        // TODO! Parallelize the rest...

        std::vector<matching::IndMatch> vec_featureMatches;
        const bool matchWorked = robustMatching(
            param,
            queryRegions,
            itMatchingImageBegin,
            itMatchingImageEnd,
            (useInputIntrinsics) ? &queryIntrinsics : nullptr,
            matchedRegions._regions,
            matchedIntrinsics,
            param._matchingError,
            param._useRobustMatching,
            param._useGuidedMatching,
            imageSize,
            std::make_pair(matchedView->ui_width, matchedView->ui_height),
            vec_featureMatches,
            param._matchingEstimator);
        if (!matchWorked) {
            OPENMVG_LOG_DEBUG("[matching]\tMatching with " << matchedView->s_Img_path << " failed! Skipping image");
            continue;
        }
        else {
            OPENMVG_LOG_DEBUG("[matching]\tFound " << vec_featureMatches.size() << " geometrically validated matches");
        }
        assert(vec_featureMatches.size() > 0);

        // if debug is enable save the matches between the query image and the current matching image
        // It saves the feature matches in a folder with the same name as the query
        // image, if it does not exist it will create it. The final svg file will have
        // a name like this: queryImage_matchedImage.svg placed in the following directory:
        // param._visualDebug/queryImage/
        if (!param._visualDebug.empty() && !imagePath.empty())
        {
            namespace bfs = boost::filesystem;
            // the current query image without extension
            const auto queryImage = bfs::path(imagePath).stem();
            // the matching image without extension
            const auto matchedImage = bfs::path(matchedView->s_Img_path).stem();
            // the full path of the matching image
            const auto matchedPath = (bfs::path(_sfm_data.s_root_path) / bfs::path(matchedView->s_Img_path)).string();

            // the directory where to save the feature matches
            const auto baseDir = bfs::path(param._visualDebug) / queryImage;
            if ((!bfs::exists(baseDir)))
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
                queryRegions.GetRegionsPositions(),
                matchedPath,
                std::make_pair(matchedView->ui_width, matchedView->ui_height),
                _regions_per_view.at(matchedView->id_view)._regions.GetRegionsPositions(),
                vec_featureMatches,
                outputName.string());
        }

        // C. recover the 2D-3D associations from the matches 
        // Each matched feature in the current similar image is associated to a 3D point
        for (const matching::IndMatch& featureMatch : vec_featureMatches) {
            // the ID of the 3D point
            const IndexT pt3D_id = matchedRegions._associated3dPoint[featureMatch._j];
            const IndexT pt2D_id = featureMatch._i;

            const auto key = std::make_pair(pt3D_id, pt2D_id);
            ++occurences[key];  // size_t is default-constructed to 0
        }
        ++goodMatches;
        if ((param._maxResults != 0) && (goodMatches == param._maxResults))
        {
            // let's say we have enough features
            OPENMVG_LOG_DEBUG("[matching]\tgot enough point from " << param._maxResults << " images");
            break;
        }
    }
    const std::size_t numCollectedPts = occurences.size();

    {
        // just debugging statistics, this block can be safely removed
        std::size_t numOccTreated = 0;
        for (std::size_t value = 1; value < numCollectedPts; ++value) {
            std::size_t counter = 0;
            for (const auto &idx : occurences)
                counter += idx.second == value;
            OPENMVG_LOG_DEBUG("[matching]\tThere are " << counter
                << " associations occurred " << value << " times ("
                << 100.0*counter / (double)numCollectedPts << "%)");
            numOccTreated += counter;
            if (numOccTreated >= numCollectedPts)
                break;
            }
        }

    pt2D = Mat2X(2, numCollectedPts);
    pt3D = Mat3X(3, numCollectedPts);

    std::size_t index = 0;
    for (const auto &idx : occurences)
    {
        // recopy all the points in the matching structure
        const IndexT pt3D_id = idx.first.first;
        const IndexT pt2D_id = idx.first.second;

        //    OPENMVG_LOG_DEBUG("[matching]\tAssociation [" << pt3D_id << "," << pt2D_id << "] occurred " << idx.second << " times");

        pt2D.col(index) = queryRegions.GetRegionPosition(pt2D_id);
        pt3D.col(index) = _sfm_data.GetLandmarks().at(pt3D_id).X;
        ++index;
    }
}

// Filter out 2NN pairs not belonging to the same image, OR not satisfying distance ratio check.
// Modifies itNNEnd so that [itNNBegin,itNNEnd) is the new range of matched descriptors.
bool KDTreeLocalizer::Filter2NN(
    const Parameters& param,
    const features::SIFT_Regions &queryRegions,
    popsift::kdtree::QueryResult::iterator& itNNBegin,
    popsift::kdtree::QueryResult::iterator& itNNEnd,
    const Reconstructed_RegionsT::RegionsT& regionsToMatch) const
{
    using namespace popsift::kdtree;
    using std::get;

    const U8Descriptor* descriptors = _kdtrees[0]->Descriptors();
    const auto reject = [&,this](const QueryResult::value_type& v) {
        if (get<1>(v).image_index != get<2>(v).image_index)
            return true;
        const U8Descriptor& nn1 = descriptors[get<1>(v).global_index];
        const U8Descriptor& nn2 = descriptors[get<2>(v).global_index];
        const U8Descriptor& q = static_cast<const U8Descriptor*>(queryRegions.DescriptorRawData())[get<0>(v)];
        unsigned d1 = L2DistanceSquared(q, nn1);
        unsigned d2 = L2DistanceSquared(q, nn2);
        return double(d2) / double(d1) >= param._fDistRatio*param._fDistRatio;
    };

    itNNEnd = std::remove_if(itNNBegin, itNNEnd, reject);
    return itNNBegin != itNNEnd;
}

bool KDTreeLocalizer::robustMatching(
    const Parameters& param,
    const features::SIFT_Regions &queryRegions,
    popsift::kdtree::QueryResult::iterator itNNBegin,
    popsift::kdtree::QueryResult::iterator itNNEnd,
    const cameras::IntrinsicBase * queryIntrinsicsBase,   // the intrinsics of the image we are using as reference
    const Reconstructed_RegionsT::RegionsT & matchedRegions,
    const cameras::IntrinsicBase * matchedIntrinsicsBase,
    const double matchingError,
    const bool robustMatching,
    const bool b_guided_matching,
    const std::pair<std::size_t,std::size_t> & imageSizeI,     // size of the first image @fixme change the API of the kernel!! 
    const std::pair<std::size_t,std::size_t> & imageSizeJ,     // size of the first image
    std::vector<matching::IndMatch> & vec_featureMatches,
    robust::EROBUST_ESTIMATOR estimator /*= robust::ROBUST_ESTIMATOR_ACRANSAC*/) const
{
  // get the intrinsics of the query camera
  if ((queryIntrinsicsBase != nullptr) && !isPinhole(queryIntrinsicsBase->getType()))
  {
    //@fixme maybe better to throw something here
    OPENMVG_CERR("[matching]\tOnly Pinhole cameras are supported!");
    return false;
  }
  if ((matchedIntrinsicsBase != nullptr) &&  !isPinhole(matchedIntrinsicsBase->getType()) )
  {
    //@fixme maybe better to throw something here
    OPENMVG_CERR("[matching]\tOnly Pinhole cameras are supported!");
    return false;
  }
  
  const cameras::Pinhole_Intrinsic *queryIntrinsics = (const cameras::Pinhole_Intrinsic*)(queryIntrinsicsBase);
  const cameras::Pinhole_Intrinsic *matchedIntrinsics = (const cameras::Pinhole_Intrinsic*)(matchedIntrinsicsBase);
  const bool canBeUndistorted = (queryIntrinsicsBase != nullptr) && (matchedIntrinsicsBase != nullptr);
    
  if (!Filter2NN(param, queryRegions, itNNBegin, itNNEnd, matchedRegions)) {
      OPENMVG_LOG_DEBUG("[matching]\Putative matching failed");
      return false;
  }

  // Populate vec_featureMatches. I: query, J: image index.
  for (auto it = itNNBegin; it != itNNEnd; ++it) {
      matching::IndMatch m;
      m._i = std::get<0>(*it);
      m._j = std::get<1>(*it).local_index;
      vec_featureMatches.push_back(m);
  }
  
  if(!robustMatching)
  {
    // nothing else to do
    return true;
  }
  
  // prepare the data for geometric filtering: for each matched pair of features,
  // store them in two matrices
  Mat featuresI(2, vec_featureMatches.size());
  Mat featuresJ(2, vec_featureMatches.size());
  // fill the matrices with the features according to vec_featureMatches
  for(int i = 0; i < vec_featureMatches.size(); ++i)
  {
    const matching::IndMatch& match = vec_featureMatches[i];
    const Vec2 &queryPoint = queryRegions.GetRegionPosition(match._i);
    const Vec2 &matchedPoint = matchedRegions.GetRegionPosition(match._j);
    
    if(canBeUndistorted)
    {
      // undistort the points for the query image
      featuresI.col(i) = queryIntrinsics->get_ud_pixel(queryPoint);
      // undistort the points for the query image
      featuresJ.col(i) = matchedIntrinsics->get_ud_pixel(matchedPoint);
    }
    else
    {
      featuresI.col(i) = queryPoint;
      featuresJ.col(i) = matchedPoint;
    }
  }
  // perform the geometric filtering
  matching_image_collection::GeometricFilter_FMatrix_AC geometricFilter(matchingError, 5000);
  std::vector<size_t> vec_matchingInliers;
  bool valid = geometricFilter.Robust_estimation(featuresI, // points of the query image
                                                 featuresJ, // points of the matched image
                                                 imageSizeI,
                                                 imageSizeJ,
                                                 vec_matchingInliers,
                                                 estimator);
  if(!valid)
  {
    OPENMVG_LOG_DEBUG("[matching]\tGeometric validation failed.");
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
    for(const std::size_t idx : vec_matchingInliers)
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
            queryIntrinsicsBase, // cameras::IntrinsicBase of the matched image
            queryRegions, // features::Regions
            matchedIntrinsicsBase, // cameras::IntrinsicBase of the query image
            matchedRegions, // features::Regions
            Square(geometricFilter.m_dPrecision_robust),
            Square(param._fDistRatio),
            vec_featureMatches); // output
  }
  return true;
}

bool KDTreeLocalizer::localizeRig(const std::vector<image::Image<unsigned char> > & vec_imageGrey,
                                   const LocalizerParameters *parameters,
                                   std::vector<cameras::Pinhole_Intrinsic_Radial_K3 > &vec_queryIntrinsics,
                                   const std::vector<geometry::Pose3 > &vec_subPoses,
                                   geometry::Pose3 &rigPose, 
                                   std::vector<LocalizationResult> & vec_locResults)
{
  const size_t numCams = vec_imageGrey.size();
  assert(numCams == vec_queryIntrinsics.size());
  assert(numCams == vec_subPoses.size() + 1);

  std::vector<std::unique_ptr<features::Regions> > vec_queryRegions(numCams);
  std::vector<std::pair<std::size_t, std::size_t> > vec_imageSize;
  
  //@todo parallelize?
  for(size_t i = 0; i < numCams; ++i)
  {
    // extract descriptors and features from each image
    vec_queryRegions[i] = std::unique_ptr<features::Regions>(new features::SIFT_Regions());
    OPENMVG_LOG_DEBUG("[features]\tExtract SIFT from query image...");
    _image_describer->Describe(vec_imageGrey[i], vec_queryRegions[i]);
    OPENMVG_LOG_DEBUG("[features]\tExtract SIFT done: found " <<  vec_queryRegions[i]->RegionCount() << " features");
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


bool KDTreeLocalizer::localizeRig(const std::vector<std::unique_ptr<features::Regions> > & vec_queryRegions,
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

bool KDTreeLocalizer::localizeRig_opengv(const std::vector<std::unique_ptr<features::Regions> > & vec_queryRegions,
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
  
  const KDTreeLocalizer::Parameters *param = static_cast<const KDTreeLocalizer::Parameters *>(parameters);
  if(!param)
  {
    // error!
    throw std::invalid_argument("The parameters are not in the right format!!");
  }

  std::vector<std::map< pair<IndexT, IndexT>, std::size_t > > vec_occurrences(numCams);
  std::vector<std::vector<voctree::DocMatch> > vec_matchedImages(numCams);
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
    auto &imageSize = vec_imageSize[camID];
    Mat &pts3D = vec_pts3D[camID];
    Mat &pts2D = vec_pts2D[camID];
    cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics = vec_queryIntrinsics[camID];
    features::SIFT_Regions &queryRegions = *dynamic_cast<features::SIFT_Regions*> (vec_queryRegions[camID].get());
    const bool useInputIntrinsics = true;
    getAllAssociations(queryRegions,
                       imageSize,
                       *param,
                       useInputIntrinsics,
                       queryIntrinsics,
                       occurrences,
                       pts2D,
                       pts3D,
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
  const bool resectionOk = rigResection(vec_pts2D,
                                        vec_pts3D,
                                        vec_queryIntrinsics,
                                        vec_subPoses,
                                        rigPose,
                                        vec_inliers,
                                        param->_angularThreshold);

  if(!resectionOk)
  {
    for(std::size_t camID = 0; camID < numCams; ++camID)
    {
      // empty result with isValid set to false
      vec_locResults.emplace_back();
    }
    OPENMVG_LOG_DEBUG("Resection failed.");
    return resectionOk;
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
    std::vector<pair<IndexT, IndexT> > indMatch3D2D;
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

      auto sqrErrors = (residuals.cwiseProduct(residuals)).colwise().sum();

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

#endif // HAVE_OPENGV

// subposes is n-1 as we consider the first camera as the main camera and the 
// reference frame of the grid
bool KDTreeLocalizer::localizeRig_naive(const std::vector<std::unique_ptr<features::Regions> > & vec_queryRegions,
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
