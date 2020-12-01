// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2015 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/feature/feature.hpp>
#include <aliceVision/feature/sift/ImageDescriber_SIFT.hpp>
#include <aliceVision/feature/akaze/ImageDescriber_AKAZE.hpp>
#include <aliceVision/matching/guidedMatching.hpp>
#include <aliceVision/multiview/relativePose/HomographyKernel.hpp>
#include <aliceVision/matching/RegionsMatcher.hpp>

#include <aliceVision/system/main.hpp>
#include <aliceVision/system/cmdline.hpp>

#include <dependencies/vectorGraphics/svgDrawer.hpp>

#include <boost/regex.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <string>
#include <iostream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace svg;
using namespace std;
using namespace aliceVision;
using namespace aliceVision::image;
using namespace aliceVision::matching;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

// Class to load images and ground truth homography matrices
// A reference image
// And a series of transformed images with the Homography mapping to the reference
class RepeatabilityDataset
{
public:
    RepeatabilityDataset(const std::string& folderPath)
        : folderPath_(folderPath)
    {
        loadImages();
        loadGroundTruthHs();
    }

    bool check() const
    {
        ALICEVISION_LOG_INFO("Dataset: " << folderPath_ << std::endl
                  << "#images: " << vec_image_.size() << "\n"
                  << "#homographies: " << vec_H_.size());
        return !vec_H_.empty() && !vec_image_.empty() && vec_H_.size() == vec_image_.size();
    }

    const image::Image<RGBColor>& image(size_t i) const { return vec_image_[i]; }
    const Mat3& H(size_t i) const { return vec_H_[i]; }
    const size_t size() const { return vec_image_.size(); }

private:
    /// Load the images of a folder
    bool loadImages()
    {
        ALICEVISION_LOG_INFO("Loading images of the dataset: " << folderPath_);

        const boost::regex ppmFilter(".*.ppm");
        const boost::regex pgmFilter(".*.pgm");

        std::vector<std::string> ppmFiles;
        std::vector<std::string> pgmFiles;

        boost::filesystem::directory_iterator endItr;
        for(boost::filesystem::directory_iterator i(folderPath_); i != endItr; ++i)
        {
            if(!boost::filesystem::is_regular_file(i->status()))
                continue;

            boost::smatch what;

            if(boost::regex_match(i->path().filename().string(), what, ppmFilter))
            {
                ppmFiles.push_back(i->path().filename().string());
                continue;
            }

            if(boost::regex_match(i->path().filename().string(), what, pgmFilter))
            {
                pgmFiles.push_back(i->path().filename().string());
            }
        }

        std::vector<std::string>& vec_image_basename = ppmFiles;

        if(!ppmFiles.empty())
            vec_image_basename = ppmFiles;
        else if(!pgmFiles.empty())
            vec_image_basename = pgmFiles;
        else
            return false;

        sort(vec_image_basename.begin(), vec_image_basename.end());
        vec_image_.resize(vec_image_basename.size());
        for(int i = 0; i < vec_image_basename.size(); ++i)
        {
            const std::string path = (fs::path(folderPath_) / vec_image_basename[i]).string();
            image::Image<RGBColor> imageRGB;
            try
            {
                image::readImage(path, imageRGB, image::EImageColorSpace::NO_CONVERSION);
                vec_image_[i] = imageRGB;
            }
            catch(std::invalid_argument& e)
            {
                image::Image<unsigned char> imageGray;
                image::readImage(path, imageGray, image::EImageColorSpace::NO_CONVERSION);
                image::ConvertPixelType(imageGray, &imageRGB);
                vec_image_[i] = imageRGB;
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
        ALICEVISION_LOG_INFO("ground truth homographies of dataset: " << folderPath_);
        vec_H_.resize(6);
        for(int i = 0; i < 6; ++i)
        {
            if(i == 0)
            {
                vec_H_[i] = Mat3::Identity();
                continue;
            }

            const std::string path = folderPath_ + "/H1to" + std::to_string(i + 1) + "p";
            std::ifstream f(path.c_str());
            if(!f.is_open())
            {
                ALICEVISION_LOG_ERROR("Unable to load ground truth homography:\n" << path);
                return false;
            }
            for(int k = 0; k < 9; ++k)
                f >> vec_H_[i].data()[k];

            vec_H_[i] /= vec_H_[i](2, 2);

            // Display
            ALICEVISION_LOG_INFO("\nGroundTruthH:\n" << vec_H_[i]);
        }
        return true;
    }

private:
    std::string folderPath_;

    std::vector<image::Image<RGBColor>> vec_image_;
    std::vector<Mat3> vec_H_;
};

/// Export point features based vector to matrices [(x,y)'T, (x,y)'T]
template <typename FeaturesT, typename MatT>
void PointsToMat(const IndMatches& matches, const FeaturesT& vec_feats0, const FeaturesT& vec_feats1, MatT& m0,
                 MatT& m1)
{
    typedef typename FeaturesT::value_type ValueT; // Container type
    typedef typename MatT::Scalar Scalar;          // Output matrix type

    m0.resize(2, matches.size());
    m1.resize(2, matches.size());

    for(size_t i = 0; i < matches.size(); ++i)
    {
        const ValueT& feat0 = vec_feats0[matches[i]._i];
        m0.col(i) << feat0.x(), feat0.y();
        const ValueT& feat1 = vec_feats1[matches[i]._j];
        m1.col(i) << feat1.x(), feat1.y();
    }
}

struct RepeatabilityResults_Matching
{
    std::map<std::string, std::vector<double>> results;

    bool exportToFile(const std::string& sFile, const std::string& sdatasetName) const
    {
        std::ofstream ofs(sFile, std::ofstream::out | std::ofstream::app);
        if(!ofs.good())
        {
            return false;
        }

        ofs << sdatasetName;
        if(!results.empty())
        {
            for(const auto a : results.begin()->second)
            {
                ofs << ";";
            }
            ofs << "\n";
            for(const auto& val : results)
            {
                const std::string sParam = val.first;
                const std::vector<double>& vec = val.second;
                ofs << sParam << ";";
                std::copy(vec.begin(), vec.end(), std::ostream_iterator<double>(ofs, ";"));
                ofs << "\n";
            }
        }
        ofs.close();

        return true;
    }
};

//--
// Regions repeatability evaluation:
// - compare feature/descriptor matching repeatability on some dataset with known homography motions
// Must be run one of the dataset contained here:
//  https://github.com/aliceVision/Features_Repeatability
//
int aliceVision_main(int argc, char** argv)
{
    std::string datasetPath;
    std::string outputFolder;
    std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);
    feature::ConfigurationPreset featDescConfig;
    bool featureRepeatability = true;
    bool matchingRepeatability = true;
    bool forceCpuExtraction = false;
    int randomSeed = std::mt19937::default_seed;
    system::EVerboseLevel verboseLevel = system::Logger::getDefaultVerboseLevel();

    po::options_description allParams("AliceVision Sample repeatabilityDataset");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input", po::value<std::string>(&datasetPath)->required(), "Path to the datasets.")
        ("output", po::value<std::string>(&outputFolder)->required(), "Output folder");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
        feature::EImageDescriberType_informations().c_str())

        ("describerPreset,p", po::value<feature::EImageDescriberPreset>(&featDescConfig.descPreset)->default_value(featDescConfig.descPreset),
        "Control the ImageDescriber configuration (low, medium, normal, high, ultra).\n"
        "Configuration 'ultra' can take long time !")
        ("describerQuality", po::value<feature::EFeatureQuality>(&featDescConfig.quality)->default_value(featDescConfig.quality),
        feature::EFeatureQuality_information().c_str())
        ("gridFiltering", po::value<bool>(&featDescConfig.gridFiltering)->default_value(featDescConfig.gridFiltering),
        "Enable grid filtering. Highly recommended to ensure usable number of features.")
        ("contrastFiltering", po::value<feature::EFeatureConstrastFiltering>(&featDescConfig.contrastFiltering)->default_value(featDescConfig.contrastFiltering),
        feature::EFeatureConstrastFiltering_information().c_str())
        ("relativePeakThreshold", po::value<float>(&featDescConfig.relativePeakThreshold)->default_value(featDescConfig.relativePeakThreshold),
        "Peak Threshold relative to median of gradiants.")
        ("forceCpuExtraction", po::value<bool>(&forceCpuExtraction)->default_value(forceCpuExtraction),
         "Use only CPU feature extraction methods.")

        ("featureRepeatability", po::value<bool>(&featureRepeatability)->default_value(featureRepeatability),
        "Feature repeatability.")
        ("matchingRepeatability", po::value<bool>(&matchingRepeatability)->default_value(matchingRepeatability),
        "MatchingRepeatability.")
        ("randomSeed", po::value<int>(&randomSeed)->default_value(randomSeed),
          "This seed value will generate a sequence using a linear random generator. Set -1 to use a random seed.")
        ;

    po::options_description logParams("Log parameters");
    logParams.add_options()("verboseLevel,v",
                            po::value<system::EVerboseLevel>(&verboseLevel)->default_value(verboseLevel),
                            "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(optionalParams).add(logParams);

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

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    std::mt19937 randomNumberGenerator(randomSeed == -1 ? std::random_device()() : randomSeed);

    //--------------------------
    // Evaluation parameters
    //--------------------------
    // - Precision radius to accept a point as a valid correspondence
    const double m_dPrecision_robust = 2.5;
    // - Nearest neighbor distance ratio to accept a photometric match between some descriptor
    const double nndr = 0.8;

    // List all subdirectories and for each one compute the repeatability
    std::vector<std::string> vec_dataset_path;

    boost::filesystem::directory_iterator endItr;
    for(boost::filesystem::directory_iterator i(datasetPath); i != endItr; ++i)
    {
        if(boost::filesystem::is_directory(i->status()))
        {
            vec_dataset_path.push_back(i->path().string());
        }
    }

    std::vector<std::shared_ptr<feature::ImageDescriber>> imageDescribers;

    // initialize feature extractor imageDescribers
    {
        std::vector<feature::EImageDescriberType> imageDescriberTypes =
            feature::EImageDescriberType_stringToEnums(describerTypesName);

        for(const auto& imageDescriberType : imageDescriberTypes)
        {
            std::shared_ptr<feature::ImageDescriber> imageDescriber = feature::createImageDescriber(imageDescriberType);
            imageDescriber->setConfigurationPreset(featDescConfig);
            if(forceCpuExtraction)
                imageDescriber->setUseCuda(false);

            imageDescribers.push_back(imageDescriber);
        }
    }

    for(const auto& imageDescriber : imageDescribers)
    {
        const std::string descName = EImageDescriberType_enumToString(imageDescriber->getDescriberType());
        ALICEVISION_LOG_INFO("Start descName: " << descName);
        for(auto const& sPath : vec_dataset_path)
        {
            ALICEVISION_LOG_INFO("Start dataset: " << sPath);

            if(fs::is_regular_file(sPath))
                continue;

            RepeatabilityDataset dataset(sPath);

            if(!dataset.check())
            {
                ALICEVISION_LOG_WARNING("Invalid dataset folder: " << sPath);
                continue;
            }

            // 1. Compute regions
            // 2. Test the repeatability (localization/overlap (accuracy))
            // 3. Export data

            using namespace aliceVision::feature;

            // For each image computes the regions:
            image::Image<float> imageGray;
            HashMap<IndexT, std::unique_ptr<Regions>> map_regions;
            for(size_t i = 0; i < dataset.size(); ++i)
            {
                image::ConvertPixelType(dataset.image(i), &imageGray);
                imageDescriber->describe(imageGray, map_regions[i]);
                ALICEVISION_LOG_INFO("image: " << i << "\t #found features: " << map_regions[i]->RegionCount());
            }

            // Repeatability evaluation to the first image
            // Evaluate the feature positions accuracy (descriptors are ignored)
            if(featureRepeatability)
            {
                RepeatabilityResults_Matching featResults;
                for(size_t i = 1; i < dataset.size(); ++i)
                {
                    if(map_regions.count(0) == 0 || map_regions.count(i) == 0)
                        continue;

                    const Regions* regions_0 = map_regions[0].get();
                    const Regions* regions_I = map_regions[i].get();
                    const PointFeatures pointsFeatures0 = regions_0->GetRegionsPositions();
                    const PointFeatures pointsFeaturesI = regions_I->GetRegionsPositions();

                    Mat x0, xI;
                    PointsToMat(pointsFeatures0, x0);
                    PointsToMat(pointsFeaturesI, xI);

                    IndMatches matches_0I;
                    matching::guidedMatching<robustEstimation::Mat3Model,
                                                multiview::relativePose::HomographyAsymmetricError>(
                        robustEstimation::Mat3Model(dataset.H(i).transpose()), x0, xI, Square(m_dPrecision_robust),
                        matches_0I);

                    ALICEVISION_LOG_INFO("Feature repeatablity Results"
                                << "\n"
                                << "*******************************"
                                << "\n"
                                << "# Keypoints 1:                        \t" << map_regions[0]->RegionCount() << "\n"
                                << "# Keypoints N:                        \t" << map_regions[i]->RegionCount() << "\n"
                                << "# Inliers:                            \t" << matches_0I.size() << "\n"
                                << "Inliers Ratio (%):                    \t"
                                << matches_0I.size() / float(map_regions[0]->RegionCount()) << "\n"
                                );

                    const std::vector<double> results = {
                        static_cast<double>(map_regions[0]->RegionCount()),
                        static_cast<double>(map_regions[i]->RegionCount()),
                        static_cast<double>(matches_0I.size()),
                        static_cast<double>(matches_0I.size() / float(map_regions[0]->RegionCount()))};
                    featResults.results[std::to_string(i)] = results;
                }
                const std::string outputFilepath =
                    (fs::path(outputFolder) / (descName + "_featureRepeatability.csv")).string();
                ALICEVISION_LOG_INFO("Export file: " << outputFilepath);
                featResults.exportToFile(outputFilepath, fs::path(sPath).stem().string());
            }

            if(matchingRepeatability)
            {
                RepeatabilityResults_Matching matchResults;
                // Test the repeatability (matching (descriptor))
                for(size_t i = 1; i < dataset.size(); ++i)
                {
                    if(map_regions.count(0) == 0 || map_regions.count(i) == 0)
                        continue;

                    const Regions* regions_0 = map_regions[0].get();
                    const Regions* regions_I = map_regions[i].get();
                    const PointFeatures pointsFeatures0 = regions_0->GetRegionsPositions();
                    const PointFeatures pointsFeaturesI = regions_I->GetRegionsPositions();

                    matching::IndMatches putativesMatches;
                    matching::DistanceRatioMatch(
                            randomNumberGenerator,
                            nndr, matching::BRUTE_FORCE_L2, *regions_0, *regions_I,
                            putativesMatches);

                    Mat x0, xI;
                    PointsToMat(putativesMatches, pointsFeatures0, pointsFeaturesI, x0, xI);

                    IndMatches matches_0I;
                    matching::guidedMatching<robustEstimation::Mat3Model,
                                                multiview::relativePose::HomographyAsymmetricError>(
                        robustEstimation::Mat3Model(dataset.H(i).transpose()), x0, xI, Square(m_dPrecision_robust),
                        matches_0I);

                    ALICEVISION_LOG_INFO("Feature matching repeatability Results"
                                << "\n"
                                << "*******************************"
                                << "\n"
                                << "** " << fs::path(sPath).stem().string() << " **"
                                << "\n"
                                << "*******************************"
                                << "\n"
                                << "# Keypoints 1:                        \t" << map_regions[0]->RegionCount() << "\n"
                                << "# Keypoints N:                        \t" << map_regions[i]->RegionCount() << "\n"
                                << "# Matches:                            \t" << putativesMatches.size() << "\n"
                                << "# Inliers:                            \t" << matches_0I.size() << "\n"
                                << "Inliers Ratio (%):                    \t"
                                << matches_0I.size() / (float)putativesMatches.size() << "\n"
                                );

                    const std::vector<double> results = {
                        static_cast<double>(map_regions[0]->RegionCount()),
                        static_cast<double>(map_regions[i]->RegionCount()),
                        static_cast<double>(putativesMatches.size()),
                        static_cast<double>(matches_0I.size()),
                        static_cast<double>(matches_0I.size() / (double)putativesMatches.size())};
                    matchResults.results[std::to_string(i)] = results;
                }
                const std::string outputFilepath = (fs::path(outputFolder) / (descName + "_matchingRepeatability.csv")).string();
                ALICEVISION_LOG_INFO("Export file: " << outputFilepath);
                matchResults.exportToFile(outputFilepath, fs::path(sPath).stem().string());
            }
        }
    }
    return EXIT_SUCCESS;
}
