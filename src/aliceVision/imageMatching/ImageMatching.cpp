// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImageMatching.hpp"
#include <aliceVision/voctree/databaseIO.hpp>

namespace aliceVision {
namespace imageMatching {

std::ostream& operator<<(std::ostream& os, const PairList& pl)
{
    for (PairList::const_iterator plIter = pl.begin(); plIter != pl.end(); ++plIter)
    {
        os << plIter->first;
        for (ImageID id : plIter->second)
        {
            os << " " << id;
        }
        os << "\n";
    }
    return os;
}

std::string EImageMatchingMethod_enumToString(EImageMatchingMethod m)
{
    switch (m)
    {
        case EImageMatchingMethod::EXHAUSTIVE:
            return "Exhaustive";
        case EImageMatchingMethod::VOCABULARYTREE:
            return "VocabularyTree";
        case EImageMatchingMethod::SEQUENTIAL:
            return "Sequential";
        case EImageMatchingMethod::SEQUENTIAL_AND_VOCABULARYTREE:
            return "SequentialAndVocabularyTree";
        case EImageMatchingMethod::FRUSTUM:
            return "Frustum";
        case EImageMatchingMethod::FRUSTUM_OR_VOCABULARYTREE:
            return "FrustumOrVocabularyTree";
    }
    throw std::out_of_range("Invalid EImageMatchingMethod enum: " + std::to_string(int(m)));
}

EImageMatchingMethod EImageMatchingMethod_stringToEnum(const std::string& m)
{
    std::string mode = m;
    std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);

    if (mode == "exhaustive")
        return EImageMatchingMethod::EXHAUSTIVE;
    if (mode == "vocabularytree")
        return EImageMatchingMethod::VOCABULARYTREE;
    if (mode == "sequential")
        return EImageMatchingMethod::SEQUENTIAL;
    if (mode == "sequentialandvocabularytree")
        return EImageMatchingMethod::SEQUENTIAL_AND_VOCABULARYTREE;
    if (mode == "frustum")
        return EImageMatchingMethod::FRUSTUM;
    if (mode == "frustumorvocabularytree")
        return EImageMatchingMethod::FRUSTUM_OR_VOCABULARYTREE;

    throw std::out_of_range("Invalid EImageMatchingMethod: " + m);
}

std::ostream& operator<<(std::ostream& os, EImageMatchingMethod m) { return os << EImageMatchingMethod_enumToString(m); }

std::istream& operator>>(std::istream& in, EImageMatchingMethod& m)
{
    std::string token;
    in >> token;
    m = EImageMatchingMethod_stringToEnum(token);
    return in;
}

std::string EImageMatchingMode_description()
{
    return "The mode to combine image matching between the input SfMData A and B: \n"
           "* a/a+a/b : A with A + A with B\n"
           "* a/ab    : A with A and B\n"
           "* a/b     : A with B\n"
           "* a/a     : A with A";
}

std::string EImageMatchingMode_enumToString(EImageMatchingMode modeMultiSfM)
{
    switch (modeMultiSfM)
    {
        case EImageMatchingMode::A_A_AND_A_B:
            return "a/a+a/b";
        case EImageMatchingMode::A_AB:
            return "a/ab";
        case EImageMatchingMode::A_B:
            return "a/b";
        case EImageMatchingMode::A_A:
            return "a/a";
    }
    throw std::out_of_range("Invalid modeMultiSfM enum");
}

EImageMatchingMode EImageMatchingMode_stringToEnum(const std::string& modeMultiSfM)
{
    std::string mode = modeMultiSfM;
    std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);  // tolower

    if (mode == "a/a+a/b")
        return EImageMatchingMode::A_A_AND_A_B;
    if (mode == "a/ab")
        return EImageMatchingMode::A_AB;
    if (mode == "a/b")
        return EImageMatchingMode::A_B;
    if (mode == "a/a")
        return EImageMatchingMode::A_A;

    throw std::out_of_range("Invalid modeMultiSfM : " + modeMultiSfM);
}

void convertAllMatchesToPairList(const PairList& allMatches, std::size_t numMatches, OrderedPairList& outPairList)
{
    outPairList.clear();

    if (numMatches == 0)
        numMatches = allMatches.size();  // disable image matching limit

    for (const auto& match : allMatches)
    {
        ImageID currImageId = match.first;
        OrderedListOfImageID bestMatches;

        for (const ImageID currMatchId : match.second)
        {
            // avoid self-matching
            if (currMatchId == currImageId)
                continue;

            // if the currMatchId ID is lower than the current image ID and
            // the current image ID is not already in the list of currMatchId
            // BOOST_ASSERT( ( currMatchId < currImageId ) &&
            //              ( outPairList.find( currMatchId ) != outPairList.end() ) );
            if (currMatchId < currImageId)
            {
                OrderedPairList::const_iterator currMatches = outPairList.find(currMatchId);
                if (currMatches != outPairList.end() && currMatches->second.find(currImageId) == currMatches->second.end())
                {
                    // then add it to the list
                    bestMatches.insert(currMatchId);
                }
            }
            else
            {
                bestMatches.insert(currMatchId);
            }

            // stop if numMatches is satisfied
            if (bestMatches.size() == numMatches)
                break;
        }

        // fill the output if we have matches
        if (!bestMatches.empty())
        {
            outPairList[currImageId] = bestMatches;
        }
    }
}

void generateSequentialMatches(const sfmData::SfMData& sfmData, size_t nbMatches, OrderedPairList& outPairList)
{
    std::vector<std::pair<std::string, IndexT>> sortedImagePaths;
    sortedImagePaths.reserve(sfmData.getViews().size());
    for (const auto& vIt : sfmData.getViews())
    {
        sortedImagePaths.emplace_back(vIt.second->getImage().getImagePath(), vIt.first);
    }
    std::sort(sortedImagePaths.begin(), sortedImagePaths.end());
    for (size_t i = 0; i < sortedImagePaths.size(); ++i)
    {
        for (size_t n = i + 1, nMax = std::min(i + nbMatches + 1, sortedImagePaths.size()); n < nMax; ++n)
        {
            size_t a = sortedImagePaths[i].second;
            size_t b = sortedImagePaths[n].second;
            outPairList[std::min(a, b)].insert(std::max(a, b));
        }
    }
}

void generateAllMatchesInOneMap(const std::set<IndexT>& viewIds, OrderedPairList& outPairList)
{
    for (const IndexT imgA : viewIds)
    {
        OrderedListOfImageID outPerImg;

        for (const IndexT imgB : viewIds)
        {
            if (imgB > imgA)
                outPerImg.insert(imgB);
        }

        if (!outPerImg.empty())
        {
            OrderedPairList::iterator itFind = outPairList.find(imgA);

            if (itFind == outPairList.end())
                outPairList[imgA] = outPerImg;
            else
                itFind->second.insert(outPerImg.begin(), outPerImg.end());
        }
    }
}

void generateAllMatchesBetweenTwoMap(const std::set<IndexT>& viewIdsA, const std::set<IndexT>& viewIdsB, OrderedPairList& outPairList)
{
    for (const IndexT imgA : viewIdsA)
    {
        OrderedListOfImageID outPerImg;

        for (const IndexT imgB : viewIdsB)
            outPerImg.insert(imgB);

        if (!outPerImg.empty())
        {
            OrderedPairList::iterator itFind = outPairList.find(imgA);

            if (itFind == outPairList.end())
                outPairList[imgA] = outPerImg;
            else
                itFind->second.insert(outPerImg.begin(), outPerImg.end());
        }
    }
}

void generateFromVoctree(PairList& allMatches,
                         const std::map<IndexT, std::string>& descriptorsFiles,
                         const aliceVision::voctree::Database& db,
                         const aliceVision::voctree::VocabularyTree<DescriptorFloat>& tree,
                         EImageMatchingMode modeMultiSfM,
                         std::size_t nbMaxDescriptors,
                         std::size_t numImageQuery)
{
    ALICEVISION_LOG_INFO("Generate matches in mode: " + EImageMatchingMode_enumToString(modeMultiSfM));

    if (numImageQuery == 0)
    {
        // if 0 retrieve the score for all the documents of the database
        numImageQuery = db.size();
    }

    // initialize allMatches

    for (const auto& descriptorPair : descriptorsFiles)
    {
        if (allMatches.find(descriptorPair.first) == allMatches.end())
            allMatches[descriptorPair.first] = {};
    }

    // query each document
#pragma omp parallel for
    for (ptrdiff_t i = 0; i < static_cast<ptrdiff_t>(descriptorsFiles.size()); ++i)
    {
        auto itA = descriptorsFiles.cbegin();
        std::advance(itA, i);
        const IndexT viewIdA = itA->first;
        const std::string featuresPathA = itA->second;

        aliceVision::voctree::SparseHistogram imageSH;

        if (modeMultiSfM != EImageMatchingMode::A_B)
        {
            // sparse histogram of A is already computed in the DB
            imageSH = db.getSparseHistogramPerImage().at(viewIdA);
        }
        else  // mode AB
        {
            // compute the sparse histogram of each image A
            std::vector<DescriptorUChar> descriptors;
            // read the descriptors
            loadDescsFromBinFile(featuresPathA, descriptors, false, nbMaxDescriptors);
            imageSH = tree.quantizeToSparse(descriptors);
        }

        std::vector<aliceVision::voctree::DocMatch> matches;

        db.find(imageSH, numImageQuery, matches);

        ListOfImageID& imgMatches = allMatches.at(viewIdA);
        imgMatches.reserve(imgMatches.size() + matches.size());

        for (const aliceVision::voctree::DocMatch& m : matches)
        {
            imgMatches.push_back(m.id);
        }
    }
}

void conditionVocTree(const std::string& treeName,
                      bool withWeights,
                      const std::string& weightsName,
                      const EImageMatchingMode matchingMode,
                      const std::vector<std::string>& featuresFolders,
                      const sfmData::SfMData& sfmDataA,
                      std::size_t nbMaxDescriptors,
                      const std::string& sfmDataFilenameA,
                      const sfmData::SfMData& sfmDataB,
                      const std::string& sfmDataFilenameB,
                      bool useMultiSfM,
                      const std::map<IndexT, std::string>& descriptorsFilesA,
                      std::size_t numImageQuery,
                      OrderedPairList& selectedPairs)
{
    if (treeName.empty())
    {
        throw std::runtime_error("No vocabulary tree argument.");
    }

    // load vocabulary tree
    ALICEVISION_LOG_INFO("Loading vocabulary tree");

    auto loadVoctree_start = std::chrono::steady_clock::now();
    aliceVision::voctree::VocabularyTree<DescriptorFloat> tree(treeName);
    auto loadVoctree_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - loadVoctree_start);
    {
        std::stringstream ss;
        ss << "tree loaded with:" << std::endl << "\t- " << tree.levels() << " levels" << std::endl;
        ss << "\t- " << tree.splits() << " branching factor" << std::endl;
        ss << "\tin " << loadVoctree_elapsed.count() << " seconds" << std::endl;
        ALICEVISION_LOG_INFO(ss.str());
    }

    // create the databases
    ALICEVISION_LOG_INFO("Creating the databases...");

    // add each object (document) to the database
    aliceVision::voctree::Database db(tree.words());
    aliceVision::voctree::Database db2;

    if (withWeights)
    {
        ALICEVISION_LOG_INFO("Loading weights...");
        db.loadWeights(weightsName);
    }
    else
    {
        ALICEVISION_LOG_INFO("No weights specified, skipping...");
    }

    if (matchingMode == EImageMatchingMode::A_A_AND_A_B)
        db2 = db;  // initialize database2 with database1 initialization

    // read the descriptors and populate the databases
    {
        std::stringstream ss;

        for (const std::string& featuresFolder : featuresFolders)
            ss << "\t- " << featuresFolder << std::endl;

        ALICEVISION_LOG_INFO("Reading descriptors from: " << std::endl << ss.str());

        std::size_t nbFeaturesLoadedInputA = 0;
        std::size_t nbFeaturesLoadedInputB = 0;
        std::size_t nbSetDescriptors = 0;

        auto detect_start = std::chrono::steady_clock::now();
        {
            if ((matchingMode == EImageMatchingMode::A_A_AND_A_B) || (matchingMode == EImageMatchingMode::A_AB) ||
                (matchingMode == EImageMatchingMode::A_A))
            {
                nbFeaturesLoadedInputA = voctree::populateDatabase<DescriptorUChar>(sfmDataA, featuresFolders, tree, db, nbMaxDescriptors);
                nbSetDescriptors = db.getSparseHistogramPerImage().size();

                if (nbFeaturesLoadedInputA == 0)
                {
                    throw std::runtime_error("No descriptors loaded in '" + sfmDataFilenameA + "'");
                }
            }

            if ((matchingMode == EImageMatchingMode::A_AB) || (matchingMode == EImageMatchingMode::A_B))
            {
                nbFeaturesLoadedInputB = voctree::populateDatabase<DescriptorUChar>(sfmDataB, featuresFolders, tree, db, nbMaxDescriptors);
                nbSetDescriptors = db.getSparseHistogramPerImage().size();
            }

            if (matchingMode == EImageMatchingMode::A_A_AND_A_B)
            {
                nbFeaturesLoadedInputB = voctree::populateDatabase<DescriptorUChar>(sfmDataB, featuresFolders, tree, db2, nbMaxDescriptors);
                nbSetDescriptors += db2.getSparseHistogramPerImage().size();
            }

            if (useMultiSfM && (nbFeaturesLoadedInputB == 0))
            {
                throw std::runtime_error("No descriptors loaded in '" + sfmDataFilenameB + "'");
            }
        }
        auto detect_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - detect_start);

        ALICEVISION_LOG_INFO("Read " << nbSetDescriptors << " sets of descriptors for a total of "
                                     << (nbFeaturesLoadedInputA + nbFeaturesLoadedInputB) << " features");
        ALICEVISION_LOG_INFO("Reading took " << detect_elapsed.count() << " sec.");
    }

    if (!withWeights)
    {
        // compute and save the word weights
        ALICEVISION_LOG_INFO("Computing weights...");

        db.computeTfIdfWeights();

        if (matchingMode == EImageMatchingMode::A_A_AND_A_B)
            db2.computeTfIdfWeights();
    }

    {
        PairList allMatches;

        ALICEVISION_LOG_INFO("Query all documents");

        auto detect_start = std::chrono::steady_clock::now();

        if (matchingMode == EImageMatchingMode::A_A_AND_A_B)
        {
            generateFromVoctree(allMatches, descriptorsFilesA, db, tree, EImageMatchingMode::A_A, nbMaxDescriptors, numImageQuery);
            generateFromVoctree(allMatches, descriptorsFilesA, db2, tree, EImageMatchingMode::A_B, nbMaxDescriptors, numImageQuery);
        }
        else
        {
            generateFromVoctree(allMatches, descriptorsFilesA, db, tree, matchingMode, nbMaxDescriptors, numImageQuery);
        }

        auto detect_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - detect_start);
        ALICEVISION_LOG_INFO("Query all documents took " << detect_elapsed.count() << " sec.");

        // process pair list
        detect_start = std::chrono::steady_clock::now();

        ALICEVISION_LOG_INFO("Convert all matches to pairList");
        convertAllMatchesToPairList(allMatches, numImageQuery, selectedPairs);
        detect_elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - detect_start);
        ALICEVISION_LOG_INFO("Convert all matches to pairList took " << detect_elapsed.count() << " sec.");
    }
}

EImageMatchingMethod selectImageMatchingMethod(EImageMatchingMethod method,
                                               const sfmData::SfMData& sfmDataA,
                                               const sfmData::SfMData& sfmDataB,
                                               std::size_t minNbImages)
{
    if (method == EImageMatchingMethod::FRUSTUM_OR_VOCABULARYTREE)
    {
        // Frustum intersection is only implemented for pinhole cameras
        bool onlyPinhole = true;
        for (auto& cam : sfmDataA.getIntrinsics())
        {
            if (!camera::isPinhole(cam.second->getType()))
            {
                onlyPinhole = false;
                break;
            }
        }

        const std::size_t reconstructedViews = sfmDataA.getValidViews().size();
        if (reconstructedViews == 0)
        {
            ALICEVISION_LOG_INFO("FRUSTUM_OR_VOCABULARYTREE: Use VOCABULARYTREE matching, as there is no known pose.");
            method = EImageMatchingMethod::VOCABULARYTREE;
        }
        else if (!onlyPinhole)
        {
            ALICEVISION_LOG_INFO("FRUSTUM_OR_VOCABULARYTREE: Use VOCABULARYTREE matching, as the scene contains non-pinhole cameras.");
            method = EImageMatchingMethod::VOCABULARYTREE;
        }
        else if (reconstructedViews == sfmDataA.getViews().size())
        {
            ALICEVISION_LOG_INFO("FRUSTUM_OR_VOCABULARYTREE: Use FRUSTUM intersection from known poses.");
            method = EImageMatchingMethod::FRUSTUM;
        }
        else
        {
            ALICEVISION_LOG_ERROR(reconstructedViews << " reconstructed views for " << sfmDataA.getViews().size() << " views.");
            throw std::runtime_error("FRUSTUM_OR_VOCABULARYTREE: Mixing reconstructed and unreconstructed Views.");
        }
    }

    // if not enough images to use the VOCABULARYTREE use the EXHAUSTIVE method
    if (method == EImageMatchingMethod::VOCABULARYTREE || method == EImageMatchingMethod::SEQUENTIAL_AND_VOCABULARYTREE)
    {
        if ((sfmDataA.getViews().size() + sfmDataB.getViews().size()) < minNbImages)
        {
            ALICEVISION_LOG_DEBUG("Use EXHAUSTIVE method instead of VOCABULARYTREE (less images than minNbImages).");
            method = EImageMatchingMethod::EXHAUSTIVE;
        }
    }
    return method;
}

}  // namespace imageMatching
}  // namespace aliceVision
