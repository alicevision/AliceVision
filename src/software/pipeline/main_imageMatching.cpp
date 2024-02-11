// This file is part of the AliceVision project.
// Copyright (c) 2015 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/matchingImageCollection/ImagePairListIO.hpp>
#include <aliceVision/imageMatching/ImageMatching.hpp>
#include <aliceVision/voctree/descriptorLoader.hpp>
#include <aliceVision/sfm/FrustumFilter.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/config.hpp>

#include <Eigen/Core>

#include <boost/program_options.hpp>

#include <iostream>
#include <filesystem>
#include <fstream>
#include <ostream>
#include <string>
#include <set>
#include <chrono>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::voctree;
using namespace aliceVision::imageMatching;

namespace po = boost::program_options;
namespace fs = std::filesystem;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    /// the file containing a list of features
    std::string sfmDataFilenameA;
    /// the folder(s) containing the extracted features with their associated descriptors
    std::vector<std::string> featuresFolders;
    /// the file in which to save the results
    std::string outputFile;

    // user optional parameters
    EImageMatchingMethod method = EImageMatchingMethod::VOCABULARYTREE;
    /// minimal number of images to use the vocabulary tree
    std::size_t minNbImages = 200;
    /// the file containing the list of features
    std::size_t nbMaxDescriptors = 500;
    /// the number of matches to retrieve for each image in Vocabulary Tree Mode
    std::size_t numImageQuery = 50;
    /// the number of neighbors to retrieve for each image in Sequential Mode
    std::size_t numImageQuerySequential = 50;
    /// the filename of the voctree
    std::string treeFilepath;
    /// the filename for the voctree weights
    std::string weightsFilepath;
    /// flag for the optional weights file
    bool withWeights = false;

    // multiple SfM parameters

    /// a second file containing a list of features
    std::string sfmDataFilenameB;
    /// the multiple SfM mode
    std::string matchingModeName = EImageMatchingMode_enumToString(EImageMatchingMode::A_A);
    /// the combine SfM output
    std::string outputCombinedSfM;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilenameA)->required(),
         "SfMData file.")
        ("featuresFolders,f", po::value<std::vector<std::string>>(&featuresFolders)->multitoken()->required(),
         "Path to folder(s) containing the extracted features.")
        ("output,o", po::value<std::string>(&outputFile)->required(),
         "Filepath to the output file with the list of selected image pairs.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("method", po::value<EImageMatchingMethod>(&method)->default_value(method),
         "Method used to select the image pairs to match:\n"
         " * VocabularyTree: select images that appear to share content\n"
         " * Sequential: use images neighbors based on filename\n"
         " * SequentialAndVocabularyTree: combine both previous approaches\n"
         " * Exhaustive: all images combinations\n"
         " * Frustum: images with camera frustum intersection (only for cameras with known poses)\n"
         " * FrustumOrVocTree: frustum intersection if cameras with known poses else use VocTree.\n")
        ("minNbImages", po::value<std::size_t>(&minNbImages)->default_value(minNbImages),
         "Minimal number of images to use the vocabulary tree. "
         "If we have less images than this threshold, we will compute all matching combinations.")
        ("maxDescriptors", po::value<std::size_t>(&nbMaxDescriptors)->default_value(nbMaxDescriptors),
         "Limit the number of descriptors you load per image. 0 means no limit.")
        ("nbMatches", po::value<std::size_t>(&numImageQuery)->default_value(numImageQuery),
         "The number of matches to retrieve for each image (If 0, it will retrieve all the matches).")
        ("nbNeighbors", po::value<std::size_t>(&numImageQuerySequential)->default_value(numImageQuerySequential),
         "The number of neighbors to retrieve for each image (If 0, it will retrieve all the neighbors).")
        ("tree,t", po::value<std::string>(&treeFilepath)->default_value(treeFilepath),
         "Input file path of the vocabulary tree. This file can be generated by 'createVoctree'. "
         "This software is intended to be used with a generic, pre-trained vocabulary tree.")
        ("weights,w", po::value<std::string>(&weightsFilepath)->default_value(weightsFilepath),
         "Input name for the vocabulary tree weight file. "
         "If not provided, all the voctree leaves will have the same weight.");

    po::options_description multiSfMParams("Multiple SfM");
    multiSfMParams.add_options()
        ("inputB", po::value<std::string>(&sfmDataFilenameB),
         "SfMData file.")
        ("matchingMode", po::value<std::string>(&matchingModeName)->default_value(matchingModeName),
         EImageMatchingMode_description().c_str())
        ("outputCombinedSfM", po::value<std::string>(&outputCombinedSfM)->default_value(outputCombinedSfM),
         "Output file path for the combined SfMData file (if empty, don't combine).");
    // clang-format on

    CmdLine cmdline("The objective of this software is to find images that are looking to the same areas of the scene. "
                    "For that, we use the image retrieval techniques to find images that share content without "
                    "the cost of resolving all feature matches in detail. The ambition is to simplify the image in "
                    "a compact image descriptor which allows to compute the distance between all images descriptors efficiently.\n"
                    "This program generates a pair list file to be passed to the aliceVision_featureMatching software. "
                    "This file contains for each image the list of most similar images.\n"
                    "AliceVision imageMatching");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    cmdline.add(multiSfMParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // multiple SfM
    const bool useMultiSfM = !sfmDataFilenameB.empty();
    const EImageMatchingMode matchingMode = EImageMatchingMode_stringToEnum(matchingModeName);

    if (useMultiSfM == (matchingMode == EImageMatchingMode::A_A))
    {
        ALICEVISION_LOG_ERROR("The number of SfMData inputs is not compatible with the selected mode.");
        return EXIT_FAILURE;
    }

    // load SfMData
    sfmData::SfMData sfmDataA, sfmDataB;

    using namespace sfmDataIO;
    if (!sfmDataIO::load(sfmDataA, sfmDataFilenameA, ESfMData(ESfMData::VIEWS | ESfMData::EXTRINSICS | ESfMData::INTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilenameA + "' cannot be read.");
        return EXIT_FAILURE;
    }

    if (useMultiSfM)
    {
        if (!sfmDataIO::load(sfmDataB, sfmDataFilenameB, ESfMData(ESfMData::VIEWS | ESfMData::EXTRINSICS | ESfMData::INTRINSICS)))
        {
            ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilenameB + "' cannot be read.");
            return EXIT_FAILURE;
        }

        // remove duplicated view
        for (const auto& viewPair : sfmDataB.getViews())
        {
            sfmData::Views::iterator it = sfmDataA.getViews().find(viewPair.first);
            if (it != sfmDataA.getViews().end())
                sfmDataA.getViews().erase(it);
        }
    }

    method = selectImageMatchingMethod(method, sfmDataA, sfmDataB, minNbImages);

    std::map<IndexT, std::string> descriptorsFilesA, descriptorsFilesB;

    if (method != EImageMatchingMethod::EXHAUSTIVE && method != EImageMatchingMethod::SEQUENTIAL)
    {
        // load descriptor filenames
        aliceVision::voctree::getListOfDescriptorFiles(sfmDataA, featuresFolders, descriptorsFilesA);

        if (useMultiSfM)
            aliceVision::voctree::getListOfDescriptorFiles(sfmDataB, featuresFolders, descriptorsFilesB);
    }

    OrderedPairList selectedPairs;

    switch (method)
    {
        case EImageMatchingMethod::EXHAUSTIVE:
        {
            ALICEVISION_LOG_INFO("Use EXHAUSTIVE method.");
            if ((matchingMode == EImageMatchingMode::A_A_AND_A_B) || (matchingMode == EImageMatchingMode::A_AB) ||
                (matchingMode == EImageMatchingMode::A_A))
                generateAllMatchesInOneMap(sfmDataA.getViewsKeys(), selectedPairs);

            if ((matchingMode == EImageMatchingMode::A_A_AND_A_B) || (matchingMode == EImageMatchingMode::A_AB) ||
                (matchingMode == EImageMatchingMode::A_B))
                generateAllMatchesBetweenTwoMap(sfmDataA.getViewsKeys(), sfmDataB.getViewsKeys(), selectedPairs);
            break;
        }
        case EImageMatchingMethod::VOCABULARYTREE:
        {
            ALICEVISION_LOG_INFO("Use VOCABULARYTREE matching.");
            conditionVocTree(treeFilepath,
                             withWeights,
                             weightsFilepath,
                             matchingMode,
                             featuresFolders,
                             sfmDataA,
                             nbMaxDescriptors,
                             sfmDataFilenameA,
                             sfmDataB,
                             sfmDataFilenameB,
                             useMultiSfM,
                             descriptorsFilesA,
                             numImageQuery,
                             selectedPairs);
            break;
        }
        case EImageMatchingMethod::SEQUENTIAL:
        {
            ALICEVISION_LOG_INFO("Use SEQUENTIAL matching.");
            generateSequentialMatches(sfmDataA, numImageQuerySequential, selectedPairs);
            break;
        }
        case EImageMatchingMethod::SEQUENTIAL_AND_VOCABULARYTREE:
        {
            ALICEVISION_LOG_INFO("Use SEQUENTIAL and VOCABULARYTREE matching.");
            generateSequentialMatches(sfmDataA, numImageQuerySequential, selectedPairs);
            conditionVocTree(treeFilepath,
                             withWeights,
                             weightsFilepath,
                             matchingMode,
                             featuresFolders,
                             sfmDataA,
                             nbMaxDescriptors,
                             sfmDataFilenameA,
                             sfmDataB,
                             sfmDataFilenameB,
                             useMultiSfM,
                             descriptorsFilesA,
                             numImageQuery,
                             selectedPairs);
            break;
        }
        case EImageMatchingMethod::FRUSTUM:
        {
            ALICEVISION_LOG_INFO("Use FRUSTUM intersection from known poses.");
            if (sfmDataA.getValidViews().empty())
            {
                throw std::runtime_error("No camera with valid pose and intrinsic.");
            }
            // For all cameras with valid extrinsic/intrinsic, we select the camera with common visibilities based on cameras' frustum.
            // We use an epsilon near value for the frustum, to ensure that mulitple images with a pure rotation will not intersect at the nodal
            // point.
            PairSet pairs = sfm::FrustumFilter(sfmDataA, 0.01).getFrustumIntersectionPairs();
            for (const auto& p : pairs)
            {
                selectedPairs[p.first].insert(p.second);
            }
            break;
        }
        case EImageMatchingMethod::FRUSTUM_OR_VOCABULARYTREE:
        {
            throw std::runtime_error("FRUSTUM_OR_VOCABULARYTREE should have been decided before.");
        }
    }

    // check if the output folder exists
    const auto basePath = fs::path(outputFile).parent_path();
    if (!basePath.empty() && !fs::exists(basePath))
    {
        // then create the missing folder
        if (!fs::create_directories(basePath))
        {
            ALICEVISION_LOG_ERROR("Unable to create folders: " << basePath);
            return EXIT_FAILURE;
        }
    }

    {
        std::size_t nbImagePairs = 0;
        for (auto& it : selectedPairs)
            nbImagePairs += it.second.size();
        ALICEVISION_LOG_INFO("Number of selected image pairs: " << nbImagePairs);
    }

    // write it to file
    PairSet selectedPairsSet;
    for (const auto& imagePairs : selectedPairs)
    {
        for (const auto& index : imagePairs.second)
        {
            selectedPairsSet.emplace(imagePairs.first, index);
        }
    }
    matchingImageCollection::savePairsToFile(outputFile, selectedPairsSet);

    ALICEVISION_LOG_INFO("pairList exported in: " << outputFile);

    if (useMultiSfM && !outputCombinedSfM.empty())
    {
        // combine A to B
        // should not loose B data
        sfmDataB.combine(sfmDataA);

        if (!sfmDataIO::save(sfmDataB, outputCombinedSfM, sfmDataIO::ESfMData::ALL))
        {
            ALICEVISION_LOG_ERROR("Unable to save combined SfM: " << outputCombinedSfM);
            return EXIT_FAILURE;
        }
    }

    return EXIT_SUCCESS;
}
