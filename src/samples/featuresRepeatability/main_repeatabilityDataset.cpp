// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/image/image.hpp"
#include "aliceVision/feature/feature.hpp"
#include "aliceVision/feature/sift/ImageDescriber_SIFT.hpp"
#include "aliceVision/feature/akaze/ImageDescriber_AKAZE.hpp"
#include "aliceVision/robustEstimation/guidedMatching.hpp"
#include "aliceVision/multiview/homographyKernelSolver.hpp"
#include "aliceVision/matching/RegionsMatcher.hpp"

#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"
#include "dependencies/vectorGraphics/svgDrawer.hpp"

#include <boost/program_options.hpp>

#include <string>
#include <iostream>

using namespace svg;
using namespace std;
using namespace aliceVision;
using namespace aliceVision::image;
using namespace aliceVision::matching;
namespace po = boost::program_options;

// Class to load images and ground truth homography matrices
// A reference image
// And a series of transformed images with the Homography mapping to the reference
class RepeatabilityDataset
{
public:
  RepeatabilityDataset
    (const std::string& folderPath)
    : folderPath_(folderPath)
  {
    loadImages();
    loadGroundTruthHs();
  }

  bool check() const {
    std::cout << "Dataset: " << folderPath_ << std::endl
     << "#images: " << vec_image_.size() << "\n"
     << "#homographies: " << vec_H_.size() << std::endl;
    return !vec_H_.empty() && !vec_image_.empty() && vec_H_.size() == vec_image_.size();
  }

  const image::Image<RGBColor>& image(size_t i) const { return vec_image_[i]; }
  const Mat3& H(size_t i) const { return vec_H_[i]; }
  const size_t size() const { return vec_image_.size(); }

private:
  /// Load the images of a directory
  bool loadImages()
  {
    std::cout << "Loading images of the dataset: " << stlplus::folder_part(folderPath_) << std::endl;
    std::vector<std::string> vec_image_basename = stlplus::folder_wildcard( folderPath_, "*.ppm" );
    if (vec_image_basename.empty())
      vec_image_basename = stlplus::folder_wildcard( folderPath_, "*.pgm" );
    if (vec_image_basename.empty())
      return false;
    sort(vec_image_basename.begin(), vec_image_basename.end());
    vec_image_.resize(vec_image_basename.size());
    for (int i = 0; i < vec_image_basename.size(); ++i)
    {
      const std::string path = stlplus::create_filespec(folderPath_, vec_image_basename[i]);
      image::Image<RGBColor> imageRGB;
      const bool bReadOk = image::ReadImage(path.c_str(), &imageRGB);
      if ( bReadOk )
      {
        vec_image_[i] = imageRGB;
      }
      else
      {
        image::Image<unsigned char> imageGray;
        if (image::ReadImage(path.c_str(), &imageGray))
        {
          image::ConvertPixelType(imageGray, &imageRGB);
          vec_image_[i] = imageRGB;
        }
        else
        {
          std::cerr << "Error: unable to load image:\n" << path << std::endl;
          return false;
        }
      }
    }
    return true;
  }

  /// Load the Homography related to each read images:
  ///  0-> Identity,
  ///  1-> H1to1p,
  ///  2-> H1to2p, ...
  bool loadGroundTruthHs()
  {
    std::cout << "ground truth homographies of dataset: " << stlplus::folder_part(folderPath_) << std::endl;
    vec_H_.resize(6);
    for (int i = 0; i < 6; ++i)
    {
      if (i == 0)
      {
        vec_H_[i] = Mat3::Identity();
        continue;
      }

      const std::string path = folderPath_ + "/H1to" + std::to_string(i+1) + "p";
      std::ifstream f(path.c_str());
      if (!f.is_open())
      {
        std::cerr << "Error: unable to load ground truth homography:\n"
             << path << std::endl;
        return false;
      }
      for (int k=0; k<9; ++k)
        f >> vec_H_[i].data()[k];

      vec_H_[i] /= vec_H_[i](2,2);

      // Display
      std::cout << "\n\n" << vec_H_[i] << std::endl;
    }
    return true;
  }

private:
  std::string folderPath_;

  std::vector<image::Image<RGBColor> > vec_image_;
  std::vector<Mat3> vec_H_;
};



/// Export point features based vector to matrices [(x,y)'T, (x,y)'T]
template< typename FeaturesT, typename MatT >
void PointsToMat(
  const IndMatches & matches,
  const FeaturesT & vec_feats0,
  const FeaturesT & vec_feats1,
  MatT & m0,
  MatT & m1)
{
  typedef typename FeaturesT::value_type ValueT; // Container type
  typedef typename MatT::Scalar Scalar; // Output matrix type

  m0.resize(2, matches.size());
  m1.resize(2, matches.size());

  for( size_t i = 0; i < matches.size(); ++i)
  {
    const ValueT & feat0 = vec_feats0[matches[i]._i];
    m0.col(i) << feat0.x(), feat0.y();
    const ValueT & feat1 = vec_feats1[matches[i]._j];
    m1.col(i) << feat1.x(), feat1.y();
  }
}

struct RepeatabilityResults_Matching
{
  std::map< std::string, std::vector<double> > results;

  bool exportToFile(const std::string & sFile, const std::string & sdatasetName) const
  {
    std::ofstream ofs(sFile, std::ofstream::out | std::ofstream::app);

    if( ! ofs.good() )
    {
        return false ;
    }

    ofs << sdatasetName << "\n";
    for ( const auto & val : results)
    {
      const std::string sParam = val.first;
      const std::vector<double> & vec = val.second;
      ofs << sParam << ";";
      std::copy(vec.begin(), vec.end(), std::ostream_iterator<double>(ofs, ";"));
      ofs << "\n";
    }
    ofs.close();

    return true ;
  }
};

//--
// Regions repeatability evaluation:
// - compare feature/descriptor matching repeatability on some dataset with known homography motions
// Must be run one of the dataset contained here:
//  https://github.com/aliceVision/Features_Repeatability
//
int main(int argc, char **argv)
{
  std::string datasetPath;
  std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
  std::string describerPreset = "NORMAL";
  bool featureRepeatability = false;
  bool matchingRepeatability = false;

  po::options_description allParams("AliceVision Sample repeatabilityDataset");
  allParams.add_options()
    ("datasetPath", po::value<std::string>(&datasetPath)->required(),
      "Path to the datasets.")
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      feature::EImageDescriberType_informations().c_str())
    ("describerPreset,p", po::value<std::string>(&describerPreset)->default_value(describerPreset),
      "Control the ImageDescriber configuration (low, medium, normal, high, ultra).\n"
      "Configuration 'ultra' can take long time !")
    ("featureRepeatability", po::value<bool>(&featureRepeatability)->default_value(featureRepeatability),
      "Feature repeatability.")
    ("matchingRepeatability", po::value<bool>(&matchingRepeatability)->default_value(matchingRepeatability),
      "MatchingRepeatability.");

  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, allParams), vm);

    if(vm.count("help") || (argc == 1))
    {
      ALICEVISION_COUT(allParams);
      return EXIT_SUCCESS;
    }
    po::notify(vm);
  }
  catch(boost::program_options::required_option& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }
  catch(boost::program_options::error& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }


  if (featureRepeatability && matchingRepeatability)
  {
    std::cerr << "Only one repeatability test can be performed at a time." << std::endl;
    return EXIT_FAILURE;
  }

  //--------------------------
  // Evaluation parameters
  //--------------------------
  // - Precision radius to accept a point as a valid correspondence
  const double m_dPrecision_robust = 2.5;
  // - Nearest neighbor distance ratio to accept a photometric match between some descriptor
  const double nndr = 0.8;

  // List all subdirectories and for each one compute the repeatability
  const std::vector<std::string> vec_dataset_path = stlplus::folder_subdirectories(datasetPath);
  for (auto const& dataset_path : vec_dataset_path)
  {
    const std::string sPath = stlplus::create_filespec(datasetPath, dataset_path);
    if (stlplus::is_file(sPath))
      continue;

    RepeatabilityDataset dataset(sPath);
    if (dataset.check())
    {
      // 1. Compute regions
      // 2. Test the repeatability (localization/overlap (accuracy))
      // 3. Export data

      using namespace aliceVision::feature;
      std::unique_ptr<ImageDescriber> image_describer;
      if (describerTypesName == "SIFT")
      {
        image_describer.reset(new ImageDescriber_SIFT(SiftParams()));
      }
      else
      if (describerTypesName == "AKAZE_FLOAT")
      {
        image_describer.reset(new ImageDescriber_AKAZE(AKAZEParams(AKAZEConfig(), AKAZE_MSURF)));
      }

      if (!image_describer)
      {
        std::cerr << "Cannot create the designed ImageDescriber:"
          << describerTypesName << "." << std::endl;
        return EXIT_FAILURE;
      }
      else
      {
        if (!image_describer->Set_configuration_preset(describerPreset))
        {
          std::cerr << "Preset configuration failed." << std::endl;
          return EXIT_FAILURE;
        }
      }

      // For each image computes the regions:
      image::Image<unsigned char> imageGray;
      HashMap<IndexT, std::unique_ptr<Regions> > map_regions;
      for (size_t i = 0; i < dataset.size(); ++i)
      {
        image::ConvertPixelType(dataset.image(i), &imageGray);
        image_describer->Describe(imageGray, map_regions[i]);
        std::cout << "image: " << i << "\t #found features: " << map_regions[i]->RegionCount() << std::endl;
      }

      // Repeatability evaluation to the first image
      // Evaluate the feature positions accuracy (descriptors are ignored)
      if (featureRepeatability)
      {
        for (size_t i = 1; i < dataset.size(); ++i)
        {
          if (map_regions.count(0) == 0 || map_regions.count(i) == 0)
            continue;

          const Regions * regions_0 = map_regions[0].get();
          const Regions * regions_I = map_regions[i].get();
          const PointFeatures pointsFeatures0 = regions_0->GetRegionsPositions();
          const PointFeatures pointsFeaturesI = regions_I->GetRegionsPositions();

          Mat x0, xI;
          PointsToMat(pointsFeatures0, x0);
          PointsToMat(pointsFeaturesI, xI);

          IndMatches matches_0I;
          robustEstimation::GuidedMatching
            <Mat3, aliceVision::homography::kernel::AsymmetricError>(
            dataset.H(i).transpose(), x0, xI, Square(m_dPrecision_robust), matches_0I);

          std::cout << "Feature repeatablity Results" << "\n"
           << "*******************************" << "\n"
           << "# Keypoints 1:                        \t" << map_regions[0]->RegionCount() << "\n"
           << "# Keypoints N:                        \t" << map_regions[i]->RegionCount() << "\n"
           << "# Inliers:                            \t" << matches_0I.size() << "\n"
           << "Inliers Ratio (%):                    \t" << matches_0I.size() / (float) map_regions[0]->RegionCount() << "\n"
           << std::endl;
        }
      }

      if (matchingRepeatability)
      {
        // Test the repeatability (matching (descriptor))
        RepeatabilityResults_Matching image_results;
        for (size_t i = 1; i < dataset.size(); ++i)
        {
          if (map_regions.count(0) == 0 || map_regions.count(i) == 0)
            continue;

          const Regions * regions_0 = map_regions[0].get();
          const Regions * regions_I = map_regions[i].get();
          const PointFeatures pointsFeatures0 = regions_0->GetRegionsPositions();
          const PointFeatures pointsFeaturesI = regions_I->GetRegionsPositions();

          matching::IndMatches putativesMatches;
          matching::DistanceRatioMatch(
            nndr, matching::BRUTE_FORCE_L2,
            *regions_0, *regions_I,
            putativesMatches);

          Mat x0, xI;
          PointsToMat(putativesMatches, pointsFeatures0, pointsFeaturesI, x0, xI);

          IndMatches matches_0I;
          robustEstimation::GuidedMatching
            <Mat3, aliceVision::homography::kernel::AsymmetricError>(
            dataset.H(i).transpose(), x0, xI, Square(m_dPrecision_robust), matches_0I);

          std::cout << "Feature matching repeatability Results" << "\n"
           << "*******************************" << "\n"
           << "** " << stlplus::basename_part(sPath) << " **" << "\n"
           << "*******************************" << "\n"
           << "# Keypoints 1:                        \t" << map_regions[0]->RegionCount() << "\n"
           << "# Keypoints N:                        \t" << map_regions[i]->RegionCount() << "\n"
           << "# Matches:                            \t" << putativesMatches.size() << "\n"
           << "# Inliers:                            \t" << matches_0I.size() << "\n"
           << "Inliers Ratio (%):                    \t" << matches_0I.size() / (float) putativesMatches.size() << "\n"
           << std::endl;

          const std::vector<double> results = {
            static_cast<double>( map_regions[0]->RegionCount() ) ,
            static_cast<double>( map_regions[i]->RegionCount() ) ,
            static_cast<double>( putativesMatches.size() ) ,
            static_cast<double>( matches_0I.size() ) ,
            static_cast<double>( matches_0I.size() / (double) putativesMatches.size())
            };
          image_results.results[std::to_string(i)] = results;
        }
        image_results.exportToFile("repeatability_results.xls", stlplus::basename_part(sPath));
      }
    }
    else
    {
      std::cerr << "Invalid dataset directory: " << dataset_path << std::endl;
      return EXIT_FAILURE;
    }
  }
  return EXIT_SUCCESS;
}
