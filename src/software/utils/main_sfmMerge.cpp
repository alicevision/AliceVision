// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/stl/hash.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/matching/IndMatch.hpp>
#include <aliceVision/matching/io.hpp>
#include <boost/program_options.hpp>
#include <aliceVision/geometry/rigidTransformation3D.hpp>
#include <aliceVision/sfm/utils/alignment.hpp>

#include <string>
#include <sstream>
#include <random>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 3
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;


namespace {

/**
 * @brief Alignment method enum
 */
enum class EMergeMethod : unsigned char
{
    SIMPLE_COPY = 0,
    FROM_LANDMARKS,
};

/**
 * @brief Convert an EMergeMethod enum to its corresponding string
 * @param[in] mergeMethod The given EMergeMethod enum
 * @return string
 */
std::string EMergeMethod_enumToString(EMergeMethod mergeMethod)
{
    switch (mergeMethod)
    {
        case EMergeMethod::SIMPLE_COPY:
            return "simple_copy";
        case EMergeMethod::FROM_LANDMARKS:
            return "from_landmarks";
    }
    throw std::out_of_range("Invalid EAlignmentMethod enum");
}

/**
 * @brief Convert a string to its corresponding EMergeMethod enum
 * @param[in] mergeMethod The given string
 * @return EMergeMethod enum
 */
EMergeMethod EMergeMethod_stringToEnum(const std::string& mergeMethod)
{
    std::string method = mergeMethod;
    std::transform(method.begin(), method.end(), method.begin(), ::tolower);  // tolower

    if (method == "simple_copy")
        return EMergeMethod::SIMPLE_COPY;
    if (method == "from_landmarks")
        return EMergeMethod::FROM_LANDMARKS;

    throw std::out_of_range("Invalid SfM merge method : " + mergeMethod);
}

inline std::istream& operator>>(std::istream& in, EMergeMethod& merge)
{
    std::string token(std::istreambuf_iterator<char>(in), {});
    merge = EMergeMethod_stringToEnum(token);
    return in;
}

inline std::ostream& operator<<(std::ostream& os, EMergeMethod e) { return os << EMergeMethod_enumToString(e); }

}  // namespace

/**
 * @brief Merge two sfmData assuming 0 duplicates. 
 * simply copy from one to another
 * @param [in, out] sfmData1 the first sfmData
 * @param [in] sfmData2 the second sfmData
 * @return true if no duplicate found
*/
bool simpleMerge(sfmData::SfMData & sfmData1, const sfmData::SfMData & sfmData2)
{
    {
        auto& views1 = sfmData1.getViews();
        auto& views2 = sfmData2.getViews();
        const size_t totalSize = views1.size() + views2.size();

        views1.insert(views2.begin(), views2.end());
        if (views1.size() < totalSize)
        {
            ALICEVISION_LOG_ERROR("Unhandled error: common view ID between both SfMData");
            return false;
        }
    }

    {
        auto& intrinsics1 = sfmData1.getIntrinsics();
        auto& intrinsics2 = sfmData2.getIntrinsics();
        const size_t totalSize = intrinsics1.size() + intrinsics2.size();

        //If both sfm share a common intrinsicId
        //Make sure there is no ambiguity and the  content is the same
        for (const auto & [key, intrinsic] : intrinsics1)
        {
            const auto & itIntrinsicOther = intrinsics2.find(key);
            if (itIntrinsicOther != intrinsics2.end())
            {
                const auto & obj1 = *intrinsic;
                const auto & obj2 = *(itIntrinsicOther->second);

                if (!(obj1 == obj2))
                {
                    ALICEVISION_LOG_ERROR("Unhandled error: common intrinsic ID with different parameters between both SfMData");
                }
            } 
        }
    }

    {
        auto& rigs1 = sfmData1.getRigs();
        auto& rigs2 = sfmData2.getRigs();
        const size_t totalSize = rigs1.size() + rigs2.size();

        rigs1.insert(rigs2.begin(), rigs2.end());
        if (rigs1.size() < totalSize)
        {
            ALICEVISION_LOG_ERROR("Unhandled error: common rigs ID between both SfMData");
            return false;
        }
    }

    {
        auto& landmarks1 = sfmData1.getLandmarks();
        auto& landmarks2 = sfmData2.getLandmarks();
        const size_t totalSize = landmarks1.size() + landmarks2.size();

        landmarks1.insert(landmarks2.begin(), landmarks2.end());
        if (landmarks1.size() < totalSize)
        {
            ALICEVISION_LOG_ERROR("Unhandled error: common rigs landmarks between both SfMData");
            return false;
        }
    }

    sfmData1.addFeaturesFolders(sfmData2.getRelativeFeaturesFolders());
    sfmData1.addMatchesFolders(sfmData2.getRelativeMatchesFolders());

    return true;
}


/**
 * @brief Merge two sfmData 
 * Align using common landmarks
 * @param [in, out] sfmData1 the first sfmData
 * @param [in] sfmData2 the second sfmData
 * @return true if no duplicate found
*/
bool fromLandmarksMerge(sfmData::SfMData & sfmData1, const sfmData::SfMData & sfmData2, const matching::PairwiseMatches & pairwiseMatches)
{
    //create map (viewId, featureId) -> landmarkId
    std::map<Pair, IndexT> mapFeatureIdToLandmarkId;
    for (const auto & plandmark : sfmData1.getLandmarks())
    {
        for (const auto & pobs : plandmark.second.getObservations())
        {
            IndexT featureId = pobs.second.getFeatureId();
            
            std::pair<IndexT, IndexT> pairViewFeature;
            pairViewFeature.first = pobs.first;
            pairViewFeature.second = featureId;
            mapFeatureIdToLandmarkId[pairViewFeature] = plandmark.first;
        }
    }

    for (const auto & plandmark : sfmData2.getLandmarks())
    {
        for (const auto & pobs : plandmark.second.getObservations())
        {
            IndexT featureId = pobs.second.getFeatureId();
            
            std::pair<IndexT, IndexT> pairViewFeature;
            pairViewFeature.first = pobs.first;
            pairViewFeature.second = featureId;

            mapFeatureIdToLandmarkId[pairViewFeature] = plandmark.first;
        }
    }
 
    std::set<std::pair<IndexT, IndexT>> landmarkUniquePairs;
    //For all pairs:
    for (const auto & pairMatches : pairwiseMatches)
    {
        Pair pairViews = pairMatches.first;

        //Is the pair of views is (view in sfmData2, view in sfmData1)
        //Or the reverse ?
        bool reverse = false;
        if (sfmData2.getViews().find(pairViews.first) == sfmData2.getViews().end())
        {
            reverse = true;
        }

        //For all types:
        for (const auto & pDescMatches: pairMatches.second)
        {
            //For all matches
            for (const auto & match : pDescMatches.second)
            {
                Pair lookup;
                
                //Check if the first feature is associated to a landmark
                lookup.first = pairViews.first;
                lookup.second = match._i;
                auto itl1 = mapFeatureIdToLandmarkId.find(lookup);
                if (itl1 == mapFeatureIdToLandmarkId.end())
                {
                    continue;
                }

                //Check if the second feature is associated to a landmark
                lookup.first = pairViews.second;
                lookup.second = match._j;
                auto itl2 = mapFeatureIdToLandmarkId.find(lookup);
                if (itl2 == mapFeatureIdToLandmarkId.end())
                {
                    continue;
                }

                std::pair<IndexT, IndexT> pairOfLandmarks;
                
                if (reverse)
                {
                    pairOfLandmarks = std::make_pair(itl1->second, itl2->second);
                }
                else 
                {
                    pairOfLandmarks = std::make_pair(itl2->second, itl1->second);
                }
                
                landmarkUniquePairs.insert(pairOfLandmarks);
            }
        }
    }

    //Transform set to vector for easier manipulation
    std::vector<std::pair<IndexT, IndexT>> landmarkPairs;
    for (const auto & pair : landmarkUniquePairs)
    {
        landmarkPairs.push_back(pair);
    }

    ALICEVISION_LOG_INFO("Matched landmarks : " << landmarkPairs.size());

    // Move input point in appropriate container
    Mat xA(3, landmarkPairs.size());
    Mat xB(3, landmarkPairs.size());
    
    int count = 0;
    for (auto & pair : landmarkPairs)
    {
        xA.col(count) = sfmData1.getLandmarks().at(pair.first).X;
        xB.col(count) = sfmData2.getLandmarks().at(pair.second).X;
        count++;
    }

    // Compute rigid transformation p'i = S R pi + t
    double S;
    Vec3 t;
    Mat3 R;
    std::vector<std::size_t> inliers;
    std::mt19937 randomNumberGenerator;

    if (!aliceVision::geometry::ACRansac_FindRTS(xA, xB, randomNumberGenerator, S, t, R, inliers, true))
    {
        ALICEVISION_LOG_INFO("Cannot found alignment");
        return false;
    }

    ALICEVISION_LOG_INFO("Inliers for SIM(3) : " << inliers.size());

    //Given inliers, create a map to translate matched landmarks from sfmData2 to sfmData1
    std::map<IndexT, IndexT> mapL2toL1;
    for (const auto & pl : sfmData2.getLandmarks())
    {
        mapL2toL1[pl.first] = UndefinedIndexT;
    }
    for (const auto & inlier : inliers)
    {
        const auto & p = landmarkPairs[inlier];
        mapL2toL1[p.second] = p.first;
    }


    // Apply found transformation on sfmData1
    sfm::applyTransform(sfmData1, S, R, t);

    ALICEVISION_LOG_INFO("First sfmData landmarks : " << sfmData1.getLandmarks().size());
    ALICEVISION_LOG_INFO("Second sfmData landmarks : " << sfmData2.getLandmarks().size());

    //Merge landmarks
    auto & landmarks1 = sfmData1.getLandmarks();
    for (const auto & pl: sfmData2.getLandmarks())
    {
        IndexT l1id = mapL2toL1[pl.first];
        if (l1id == UndefinedIndexT)
        {
            landmarks1.insert(pl);
        }
        else 
        {
            auto & obs1 = landmarks1[l1id].getObservations();
            const auto & obs2 = pl.second.getObservations();
            obs1.insert(obs2.begin(), obs2.end());
        }
    }

    ALICEVISION_LOG_INFO("Result sfmData landmarks : " << sfmData1.getLandmarks().size());
    
    
    // Simple merge of views
    auto& views1 = sfmData1.getViews();
    auto& views2 = sfmData2.getViews();
    size_t totalSize = views1.size() + views2.size();
    views1.insert(views2.begin(), views2.end());
    if (views1.size() != totalSize)
    {
        ALICEVISION_LOG_ERROR("Non Unique views");
        return false;
    }

    // Simple merge of intrinsics
    auto& intrinsics1 = sfmData1.getIntrinsics();
    auto& intrinsics2 = sfmData2.getIntrinsics();
    totalSize = intrinsics1.size() + intrinsics2.size();
    intrinsics1.insert(intrinsics2.begin(), intrinsics2.end());

    // Simple merge of poses
    auto& poses1 = sfmData1.getPoses();
    auto& poses2 = sfmData2.getPoses();
    totalSize = poses1.size() + poses2.size();
    poses1.insert(poses2.begin(), poses2.end());

    sfmData1.addFeaturesFolders(sfmData2.getRelativeFeaturesFolders());
    sfmData1.addMatchesFolders(sfmData2.getRelativeMatchesFolders());

    return true;
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::vector<std::string> sfmDataFilenames;
    std::string outSfMDataFilename;
    EMergeMethod mergeMethod = EMergeMethod::SIMPLE_COPY;
    std::vector<std::string> matchesFolders;
    std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("inputs,i", po::value<std::vector<std::string>>(&sfmDataFilenames)->multitoken(),
         "Path to sfmDatas to merge.")
        ("output,o", po::value<std::string>(&outSfMDataFilename)->required(),
         "Output SfMData scene.");
        
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("method", po::value<EMergeMethod>(&mergeMethod)->default_value(mergeMethod),
         "Alignment Method:\n"
         "\t- simple_copy: Just merge.\n"
         "\t- from_landmarks: Alilgn using landmarks sharing features.\n")
        ("matchesFolders,m", po::value<std::vector<std::string>>(&matchesFolders)->multitoken(),
         "Path to folder(s) in which computed matches are stored.")
        ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
         feature::EImageDescriberType_informations().c_str());
    // clang-format on

    CmdLine cmdline("AliceVision sfmMerge");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    if (sfmDataFilenames.empty())
    {
        ALICEVISION_LOG_ERROR("At least one sfmData input should be given.");
        return EXIT_FAILURE;
    }

    sfmData::SfMData outputSfmData;

    for (int id = 0; id < sfmDataFilenames.size(); id++)
    {
        const std::string & filename = sfmDataFilenames[id];
        ALICEVISION_LOG_INFO("Processing " << filename);

        // Load input scene
        sfmData::SfMData sfmData;
        if (!sfmDataIO::load(sfmData, filename, sfmDataIO::ESfMData::ALL))
        {
            ALICEVISION_LOG_ERROR("The input SfMData file '" << filename << "' cannot be read");
            return EXIT_FAILURE;
        }

        if (id == 0)
        {
            outputSfmData = sfmData;
            continue;
        }

        if (mergeMethod == EMergeMethod::SIMPLE_COPY)
        {
            if (!simpleMerge(outputSfmData, sfmData))
            {
                return EXIT_FAILURE;
            }
        }
        else 
        {
            // get imageDescriber type
            const std::vector<feature::EImageDescriberType> describerTypes = feature::EImageDescriberType_stringToEnums(describerTypesName);

            // matches reading
            matching::PairwiseMatches pairwiseMatches;
            if (!matching::Load(pairwiseMatches, std::set<IndexT>(), matchesFolders, describerTypes, 0, 0))
            {
                std::stringstream ss("Unable to read the matches file(s) from:\n");
                for (const std::string& folder : matchesFolders)
                {
                    ss << "\t- " << folder << "\n";
                }

                ALICEVISION_LOG_WARNING(ss.str());
                
                return EXIT_FAILURE;
            }

            if (!fromLandmarksMerge(outputSfmData, sfmData, pairwiseMatches))
            {
                return EXIT_FAILURE;
            }
        }
    }
  

    if (!sfmDataIO::save(outputSfmData, outSfMDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("An error occurred while trying to save '" << outSfMDataFilename << "'");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
