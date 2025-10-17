// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Parallelization.hpp>

#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>
#include <aliceVision/sfm/pipeline/pairwiseMatchesIO.hpp>

#include <aliceVision/robustEstimation/estimators.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterType.hpp>
#include <aliceVision/matching/matchesFiltering.hpp>

#include <aliceVision/matchingImageCollection/GeometricFilter.hpp>
#include <aliceVision/matchingImageCollection/GeometricFilterMatrix_F_AC.hpp>
#include <aliceVision/system/ProgressDisplay.hpp>
#include <boost/program_options.hpp>
#include <sstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
namespace po = boost::program_options;
namespace fs = std::filesystem;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string outputMatchesFolder;
    std::string filtersFolder;
    std::vector<std::string> featuresFolders;
    std::vector<std::string> matchesFolders;
    const std::string fileExtension = "txt";

    size_t numMatchesToKeep = 0;

    std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);

    int rangeIteration = 0;
    int rangeBlocksCount = 1;
    int randomSeed = std::mt19937::default_seed;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&outputMatchesFolder)->required(),
         "Path to a folder in which computed matches will be stored.")
        ("filters", po::value<std::string>(&filtersFolder)->required(),
         "Filters Folder.")
        ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken(),
         "Path to folder(s) containing the extracted features.")
        ("matchesFolders,m", po::value<std::vector<std::string>>(&matchesFolders)->multitoken(),
         "Path to folder(s) in which computed matches are stored.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
         feature::EImageDescriberType_informations().c_str())
        ("maxMatches", po::value<std::size_t>(&numMatchesToKeep)->default_value(numMatchesToKeep),
         "Maximum number pf matches to keep.")
        ("rangeIteration", po::value<int>(&rangeIteration)->default_value(rangeIteration), "Range current iteration.")
        ("rangeBlocksCount", po::value<int>(&rangeBlocksCount)->default_value(rangeBlocksCount), "Range number of blocks.")
        ("randomSeed", po::value<int>(&randomSeed)->default_value(randomSeed),
         "This seed value will generate a sequence using a linear random generator. Set -1 to use a random seed.");;
    // clang-format on

    CmdLine cmdline("AliceVision geometricFilterApplying");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    std::mt19937 randomNumberGenerator(randomSeed == -1 ? std::random_device()() : randomSeed);

    // load input SfMData scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS | sfmDataIO::EXTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    if (describerTypesName.empty())
    {
        ALICEVISION_LOG_ERROR("Empty option: --describerMethods");
        return EXIT_FAILURE;
    }

    const std::vector<feature::EImageDescriberType> describerTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

    // matches reading.
    // We read everything even if we only process a sub part.
    matching::PairwiseMatches pairwiseMatches;
    ALICEVISION_LOG_INFO("Load features matches");
    if (!sfm::loadPairwiseMatches(pairwiseMatches, sfmData, matchesFolders, describerTypes, numMatchesToKeep, 0, false))
    {
        ALICEVISION_LOG_ERROR("Unable to load matches.");
        return EXIT_FAILURE;
    }

    int chunkStart, chunkEnd;
    if (!rangeComputation(chunkStart, chunkEnd, rangeIteration, rangeBlocksCount, pairwiseMatches.size()))
    {
        ALICEVISION_LOG_INFO("Nothing to compute in this chunk");
    }

    // We want to process only a subset of the pairs
    // Assuming range start lies between [0 and pairwiseMatches.size()]
    matching::PairwiseMatches filteredMatches;
    std::set<IndexT> filter;

    int pos = 0;
    for (const auto& [pair, content] : pairwiseMatches)
    {
        if (pos == chunkEnd)
        {
            break;
        }

        if (pos >= chunkStart)
        {
            filteredMatches[pair] = content;
            filter.insert(pair.first);
            filter.insert(pair.second);
        }

        pos++;
    }

    ALICEVISION_LOG_INFO("A total of " << pairwiseMatches.size() << " pairs has to be processed.");
    ALICEVISION_LOG_INFO("Current chunk will analyze pairs from " << chunkStart << " to " << chunkEnd << ".");

    pairwiseMatches.clear();

    // features reading
    feature::RegionsPerView regionPerView;
    if (!sfm::loadRegionsPerView(regionPerView, sfmData, featuresFolders, describerTypes, filter))
    {
        ALICEVISION_LOG_ERROR("Invalid regions in '" + sfmDataFilename + "'");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO(std::to_string(pairwiseMatches.size()) << " putative image pair matches");

    for (const auto& imageMatch : pairwiseMatches)
    {
        ALICEVISION_LOG_INFO("\t- image pair (" << imageMatch.first.first + ", " << imageMatch.first.second << ") contains "
                                                << imageMatch.second.getNbAllMatches() << " putative matches.");
    }

    matchingImageCollection::PairwiseGeometricInfo geometricInfos;
    std::stringstream ss;
    ss << filtersFolder << "/matches_" << std::to_string(rangeIteration) << ".json";
    ALICEVISION_LOG_INFO("Loading from " << ss.str());
    if (!matchingImageCollection::loadGeometricInfos(ss.str(), geometricInfos))
    {
        ALICEVISION_LOG_ERROR("Impossible to save geometricInfos");
        return EXIT_FAILURE;
    }

    int count = 0;
    matching::PairwiseMatches finalMatches;

    ALICEVISION_LOG_INFO("Processing all pairs");
    auto progressDisplay = system::createConsoleProgressDisplay(filteredMatches.size(), std::cout);
    for (const auto& [pair, perDesc] : filteredMatches)
    {
        const auto it = geometricInfos.find(pair);
        if (it == geometricInfos.end())
        {
            progressDisplay += 1;
            continue;
        }

        const auto geometricInfo = it->second;

        robustEstimation::Mat3Model model(geometricInfo.model);
        multiview::relativePose::FundamentalEpipolarDistanceError errorFunc;

        const IndexT idViewI = pair.first;
        const IndexT idViewJ = pair.second;
        const auto& viewI = sfmData.getView(idViewI);
        const auto& viewJ = sfmData.getView(idViewJ);
        const auto& camI = sfmData.getIntrinsicSharedPtr(viewI.getIntrinsicId());
        const auto& camJ = sfmData.getIntrinsicSharedPtr(viewJ.getIntrinsicId());

        for (const auto& [desc, matches] : perDesc)
        {
            const auto& featuresI = regionPerView.getRegions(idViewI, desc).Features();
            const auto& featuresJ = regionPerView.getRegions(idViewJ, desc).Features();

            matching::IndMatches copied;
            for (const auto& match : matches)
            {
                const auto& ptI = featuresI[match._i].coords().cast<double>();
                const auto& ptJ = featuresJ[match._j].coords().cast<double>();

                if (sqrt(errorFunc.error(model, ptI, ptJ)) < geometricInfo.threshold)
                {
                    copied.push_back(match);
                }
            }

            if (copied.size() == 0)
            {
                continue;
            }

            finalMatches[pair][desc] = copied;
        }

        progressDisplay += 1;
    }

    for (const auto& imageMatch : finalMatches)
    {
        ALICEVISION_LOG_INFO("\t- image pair (" << imageMatch.first.first + ", " << imageMatch.first.second << ") contains "
                                                << imageMatch.second.getNbAllMatches() << " filtered matches.");
    }

    // export geometric filtered matches
    ALICEVISION_LOG_INFO("Save geometric matches.");

    const std::string filePrefix = std::to_string(rangeIteration) + ".";
    Save(finalMatches, outputMatchesFolder, fileExtension, false, filePrefix);

    return EXIT_SUCCESS;
}
