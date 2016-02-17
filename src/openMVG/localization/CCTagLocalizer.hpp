#pragma once

#include "ILocalizer.hpp"
#include "LocalizationResult.hpp"
#include "VoctreeLocalizer.hpp"

#include <openMVG/features/features.hpp>
#include <openMVG/features/image_describer.hpp>
#include <openMVG/features/cctag/CCTAG_describer.hpp>
#include <openMVG/sfm/sfm_data.hpp>
#include <openMVG/sfm/pipelines/localization/SfM_Localizer.hpp>

#include <iostream>
#include <bitset>

namespace openMVG {
namespace localization {

typedef Reconstructed_Regions<features::SIOPointFeature, unsigned char, 128> Reconstructed_RegionsCCTag; 
typedef Reconstructed_RegionsCCTag::DescriptorT CCTagDescriptor;
typedef Reconstructed_RegionsCCTag::FeatureT CCTagKeypoint;
typedef Hash_Map<IndexT, Reconstructed_RegionsCCTag > CCTagRegionsPerViews;

class CCTagLocalizer : public ILocalizer
{
  
  public:
  struct Parameters : LocalizerParameters
  {

    Parameters() : LocalizerParameters(), 
      _nNearestKeyFrames(4) { }
    
    size_t _nNearestKeyFrames;         //< number of best matching images to retrieve from the database                
  };
  
public:
  
  CCTagLocalizer(const std::string &sfmFilePath,
                 const std::string &descriptorsFolder);
   
 /**
   * @brief Just a wrapper around the different localization algorithm, the algorith
   * used to localized is chosen using \p param._algorithm
   * 
   * @param[in] imageGrey The input greyscale image.
   * @param[in] param The parameters for the localization.
   * @param[in] useInputIntrinsics Uses the \p queryIntrinsics as known calibration.
   * @param[in,out] queryIntrinsics Intrinsic parameters of the camera, they are used if the
   * flag useInputIntrinsics is set to true, otherwise they are estimated from the correspondences.
   * @param[out] localizationResult The localization result containing the pose and the associations.
   * @param[in] imagePath Optional complete path to the image, used only for debugging purposes.
   * @return  true if the image has been successfully localized.
   */
  bool localize(const image::Image<unsigned char> & imageGrey,
                const LocalizerParameters *parameters,
                bool useInputIntrinsics,
                cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                LocalizationResult & localizationResult, const std::string& imagePath = std::string());

  bool localize(const std::unique_ptr<features::Regions> &queryRegions,
                const std::pair<std::size_t, std::size_t> imageSize,
                const LocalizerParameters *parameters,
                bool useInputIntrinsics,
                cameras::Pinhole_Intrinsic_Radial_K3 &queryIntrinsics,
                LocalizationResult & localizationResult,
                const std::string& imagePath = std::string());
  
  bool localizeRig(const std::vector<image::Image<unsigned char> > & vec_imageGrey,
                const LocalizerParameters *parameters,
                std::vector<cameras::Pinhole_Intrinsic_Radial_K3 > &vec_queryIntrinsics,
                const std::vector<geometry::Pose3 > &vec_subPoses,
                geometry::Pose3 rigPose);
  
  
  // @todo UNSTABLE / TO FIX
  bool localize(const std::vector<std::unique_ptr<features::Regions> > & vec_queryRegions,
              const Parameters &param,
              const std::vector<cameras::Pinhole_Intrinsic_Radial_K3 > &vec_queryIntrinsics,
              const std::vector<geometry::Pose3 > &vec_subPoses,
              geometry::Pose3 rigPose);
  
  bool localizeAllAssociations(const std::vector<std::unique_ptr<features::Regions> > & vec_queryRegions,
                const Parameters &param,
                const std::vector<cameras::Pinhole_Intrinsic_Radial_K3 > &vec_queryIntrinsics,
                const std::vector<geometry::Pose3 > &vec_subPoses,
                geometry::Pose3 rigPose);
 
  virtual ~CCTagLocalizer();
   
  void getAllAssociationsFromNearestKFrames(const features::CCTAG_Regions &queryRegions,
                                            const CCTagLocalizer::Parameters &param,
                                            std::map< pair<IndexT, IndexT>, pair<Vec3, Vec2> > &associations) const;
  
private:
  
  bool loadReconstructionDescriptors(
    const sfm::SfM_Data & sfm_data,
    const std::string & feat_directory);
  
  // for each view index, it contains the cctag features and descriptors that have an
  // associated 3D point
  CCTagRegionsPerViews _regions_per_view;
   
  // the feature extractor
  features::CCTAG_Image_describer _image_describer;
  
  //
  //std::map<IndexT, Vec3> _cctagDatabase;
};

 /**
   * @brief Retrieve the k nearest views in a collection of views based on a query
   *        consisting in a set of CCTag regions.
   * 
   * @param[in] queryRegions Set of CCTag regions in the query.
   * @param[in] Collection of views containing a set of cctag regions.
   * @param[in] nNearestKeyFrames Number of nearest neighbours to return.
   * @param[out] kNearestFrames Set of computed indices associated to the k nearest views.
   * @param[in] similarityThreshold A threshold to retrieve only the kframes having 
  *  at least \p similarityThreshold similarity score.
   */
void kNearestKeyFrames(
          const features::CCTAG_Regions & queryRegions,
          const CCTagRegionsPerViews & _regions_per_view,
          std::size_t nNearestKeyFrames,
          std::vector<IndexT> & kNearestFrames,
          const float similarityThreshold = .0f);
/**
 * @brief Given a set of CCTag descriptors seen in a view, it creates a descriptor for the view: the
 * view descriptor is a 128 bit array (ie the number of possible markers) whose 
 * bits are 0 or 1 whether the corresponding marker is seen or not. E.g. if the 
 * bit in position 8 is 1 it means that the marker with ID 8 has been seen by the view.
 * 
 * @param[in] vCCTagDescriptors The input descriptors associated to the view.
 * @return The view descriptor as a set of bit representing the visibility of
 * each possible marker for that view.
 */
std::bitset<128> constructCCTagViewDescriptor(
        const std::vector<CCTagDescriptor> & vCCTagDescriptors);

float viewSimilarity(
        const features::CCTAG_Regions & regionsA,
        const features::CCTAG_Regions & regionsB);

void viewMatching(
        const features::CCTAG_Regions & regionsA,
        const features::CCTAG_Regions & regionsB,
        std::vector<matching::IndMatch> & vec_featureMatches);

} // namespace localization
} // openMVG

