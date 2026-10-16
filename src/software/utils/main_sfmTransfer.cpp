// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/utils/alignment.hpp>
#include <aliceVision/sfm/utils/sfmTransfer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/program_options.hpp>

#include <string>
#include <utility>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 2

using namespace aliceVision;
using namespace aliceVision::sfm;

namespace po = boost::program_options;

/*
 * sfmTransfer copies data from a reference SfMData onto a destination SfMData.
 *
 * The views of the destination are never added nor removed, and rigs are never created.
 *
 * With the from_viewid, from_filepath and from_metadata methods, destination views are first matched
 * one-to-one with reference views (see findMatchingViews), then each enabled component is transferred:
 *  - poses (transferMatchingPoses): rig-aware copy, existing poses are overwritten;
 *  - intrinsics (transferMatchingIntrinsics): destination intrinsics are overwritten, or split if
 *    their views match several reference intrinsics, and never merged;
 *  - landmarks (transferMatchingLandmarks): destination landmarks are replaced by the reference ones;
 *  - survey points (transferMatchingSurveyPoints): replaced per matched view.
 * Destination views which are not matched keep their data, except that they share any overwritten intrinsic.
 *
 * With the from_poseid and from_intrinsicid methods, no view matching is done and the transfer options
 * are ignored: only the poses (resp. intrinsics) with the same ID in both scenes are overwritten.
 *
 * In all cases, unused poses, intrinsics and landmarks without observations are removed before saving.
 */

/**
 * @brief Matching Views method enum
 */
enum class EMatchingMethod : unsigned char
{
    FROM_VIEWID = 0,
    FROM_FILEPATH,
    FROM_METADATA,
    FROM_POSEID,
    FROM_INTRINSICID
};

/**
 * @brief Convert an EMatchingMethod enum to its corresponding string
 * @param[in] matchingMethod The given EMatchingMethod enum
 * @return string
 */
std::string EMatchingMethod_enumToString(EMatchingMethod alignmentMethod)
{
    switch (alignmentMethod)
    {
        case EMatchingMethod::FROM_VIEWID:
            return "from_viewid";
        case EMatchingMethod::FROM_FILEPATH:
            return "from_filepath";
        case EMatchingMethod::FROM_METADATA:
            return "from_metadata";
        case EMatchingMethod::FROM_POSEID:
            return "from_poseid";
        case EMatchingMethod::FROM_INTRINSICID:
            return "from_intrinsicid";
    }
    throw std::out_of_range("Invalid EMatchingMethod enum");
}

/**
 * @brief Convert a string to its corresponding EMatchingMethod enum
 * @param[in] matchingMethod The given string
 * @return EMatchingMethod enum
 */
EMatchingMethod EMatchingMethod_stringToEnum(const std::string& alignmentMethod)
{
    std::string method = alignmentMethod;
    std::transform(method.begin(), method.end(), method.begin(), ::tolower);  // tolower

    if (method == "from_viewid")
        return EMatchingMethod::FROM_VIEWID;
    if (method == "from_filepath")
        return EMatchingMethod::FROM_FILEPATH;
    if (method == "from_metadata")
        return EMatchingMethod::FROM_METADATA;
    if (method == "from_poseid")
        return EMatchingMethod::FROM_POSEID;
    if (method == "from_intrinsicid")
        return EMatchingMethod::FROM_INTRINSICID;

    throw std::out_of_range("Invalid SfM alignment method : " + alignmentMethod);
}

inline std::istream& operator>>(std::istream& in, EMatchingMethod& alignment)
{
    std::string token(std::istreambuf_iterator<char>(in), {});
    alignment = EMatchingMethod_stringToEnum(token);
    return in;
}

inline std::ostream& operator<<(std::ostream& os, EMatchingMethod e) { return os << EMatchingMethod_enumToString(e); }

/**
 * @brief Build the (destination view ID, reference view ID) pairs of matching views.
 *        - FROM_VIEWID: views with the same view ID in both scenes.
 *        - FROM_FILEPATH: views whose image paths give the same key, the key being the concatenation of the
 *          groups captured by fileMatchingPattern (or the full path if the pattern is empty).
 *        - FROM_METADATA: views whose concatenated values of the metadataMatchingList entries are equal.
 *        For FROM_FILEPATH and FROM_METADATA, a key shared by several views of one scene is ambiguous and
 *        all these views are discarded, so the matching is always one-to-one. Views with an empty key
 *        (path not matching the pattern, or none of the metadata) are never matched.
 * @param[in] sfmData The destination SfMData
 * @param[in] sfmDataRef The reference SfMData
 * @param[in] matchingMethod The matching method (FROM_POSEID and FROM_INTRINSICID are not supported here)
 * @param[in] fileMatchingPattern Regex used by FROM_FILEPATH
 * @param[in] metadataMatchingList Metadata keys used by FROM_METADATA
 * @param[out] commonViewIds Matching (destination view ID, reference view ID) pairs
 * @return false if the matching method is not supported
 */
bool findMatchingViews(const sfmData::SfMData& sfmData,
                       const sfmData::SfMData& sfmDataRef,
                       EMatchingMethod matchingMethod,
                       const std::string& fileMatchingPattern,
                       const std::vector<std::string>& metadataMatchingList,
                       std::vector<std::pair<IndexT, IndexT>>& commonViewIds)
{
    commonViewIds.clear();

    if (matchingMethod == EMatchingMethod::FROM_VIEWID)
    {
        // Get common views by view ID
        for (IndexT id : sfmData.getCommonViews(sfmDataRef))
        {
            commonViewIds.emplace_back(id, id);
        }
    }
    else if (matchingMethod == EMatchingMethod::FROM_FILEPATH)
    {
        // Get common views by file path pattern
        sfm::matchViewsByFilePattern(sfmData, sfmDataRef, fileMatchingPattern, commonViewIds);
    }
    else if (matchingMethod == EMatchingMethod::FROM_METADATA)
    {
        // Get common views by metadata matching
        sfm::matchViewsByMetadataMatching(sfmData, sfmDataRef, metadataMatchingList, commonViewIds);
    }
    else
    {
        ALICEVISION_LOG_ERROR("Unsupported matching method for matching views.");
        return false;
    }

    return true;
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string outSfMDataFilename;
    std::string sfmDataReferenceFilename;
    bool transferPoses = true;
    bool transferIntrinsics = true;
    bool transferLandmarks = true;
    bool transferSurveyPoints = true;
    EMatchingMethod matchingMethod = EMatchingMethod::FROM_VIEWID;
    std::string fileMatchingPattern;
    std::vector<std::string> metadataMatchingList = {"Make", "Model", "Exif:BodySerialNumber", "Exif:LensSerialNumber"};
    std::string outputViewsAndPosesFilepath;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "Path to the destination SfMData file. This is the SfM scene onto which the camera poses, intrinsics, and landmarks will be transferred.")
        ("output,o", po::value<std::string>(&outSfMDataFilename)->required(),
         "Output SfMData scene.")
        ("reference,r", po::value<std::string>(&sfmDataReferenceFilename)->required(),
         "Path to the reference SfMData file used to retrieve resolved poses, intrinsics and landmarks.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("method", po::value<EMatchingMethod>(&matchingMethod)->default_value(matchingMethod),
         "Matching method:\n"
         "\t- from_viewid: Match views with same view ID.\n"
         "\t- from_filepath: Match views with a filepath matching, using --fileMatchingPattern.\n"
         "\t- from_metadata: Match views with matching metadata, using --metadataMatchingList.\n"
         "\t- from_poseid: Update poses with the same pose ID. Only the poses will be updated.\n"
         "\t- from_intrinsicid: Update intrinsics with the same intrinsic ID. Only the intrinsics will be updated.\n"
         "The from_poseid and from_intrinsicid methods ignore the transfer options.\n")
        ("fileMatchingPattern", po::value<std::string>(&fileMatchingPattern)->default_value(fileMatchingPattern),
         "Matching regex for the from_filepath method. The captured groups are concatenated to build the matching key.\n"
         "If empty, the full image path is used. Keys shared by several views of a scene are ignored.\n")
        ("metadataMatchingList", po::value<std::vector<std::string>>(&metadataMatchingList)->multitoken()->default_value(metadataMatchingList),
         "List of metadata that should match to create the correspondences (from_metadata method).\n"
         "Values shared by several views of a scene are ignored.\n")
        ("transferPoses", po::value<bool>(&transferPoses)->default_value(transferPoses),
         "Transfer poses of the reconstructed reference views onto the matched views. "
         "The rig structure is kept if both views are rig-dependent, otherwise the view gets an independent pose.")
        ("transferIntrinsics", po::value<bool>(&transferIntrinsics)->default_value(transferIntrinsics),
         "Transfer intrinsics of the reconstructed reference views onto the matched views. "
         "A destination intrinsic is overwritten, or split if its views match different reference intrinsics.")
        ("transferLandmarks", po::value<bool>(&transferLandmarks)->default_value(transferLandmarks),
         "Replace the destination landmarks by the reference landmarks observed in matched views. "
         "Nothing is done if the reference scene has no landmarks.")
        ("transferSurveyPoints", po::value<bool>(&transferSurveyPoints)->default_value(transferSurveyPoints),
         "Replace the survey points of each matched view by the ones of its reference view, if any.")
        ("outputViewsAndPoses", po::value<std::string>(&outputViewsAndPosesFilepath),
         "Path to the output SfMData file with cameras (views and poses).");
    // clang-format on

    CmdLine cmdline("AliceVision sfmTransfer");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    if (!transferPoses && !transferIntrinsics && !transferLandmarks && !transferSurveyPoints)
    {
        ALICEVISION_LOG_ERROR("Nothing to do: all transfer options are disabled.");
        return EXIT_FAILURE;
    }

    // Load input scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
        return EXIT_FAILURE;
    }

    // Load reference scene
    sfmData::SfMData sfmDataRef;
    if (!sfmDataIO::load(sfmDataRef, sfmDataReferenceFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The reference SfMData file '" << sfmDataReferenceFilename << "' cannot be read");
        return EXIT_FAILURE;
    }

    if (matchingMethod == EMatchingMethod::FROM_INTRINSICID)
    {
        // Copy matching intrinsics from the reference SfMData to the input SfMData.
        // Intrinsics are matched by ID only, views and other components are left untouched.
        for (auto& [idIntrinsic, intrinsic] : sfmData.getIntrinsics())
        {
            const auto intrinsicRef = sfmDataRef.getIntrinsics().find(idIntrinsic);
            if (intrinsicRef != sfmDataRef.getIntrinsics().end())
            {
                intrinsic.reset(intrinsicRef->second->clone());
            }
        }
    }
    else if (matchingMethod == EMatchingMethod::FROM_POSEID)
    {
        // Copy matching poses from the reference SfMData to the input SfMData.
        // Poses are matched by ID only, without any rig handling. Other components are left untouched.
        for (auto& view : sfmData.getViews())
        {
            auto pose = sfmDataRef.getPoses().find(view.second->getPoseId());
            if (pose != sfmDataRef.getPoses().end())
            {
                view.second->setPoseId(pose->first);
                sfmData.getPoses().assign(pose->first, *(pose->second));
            }
        }
    }
    else
    {
        ALICEVISION_LOG_INFO("Search common views.");

        std::vector<std::pair<IndexT, IndexT>> commonViewIds;
        if (!findMatchingViews(sfmData, sfmDataRef, matchingMethod, fileMatchingPattern, metadataMatchingList, commonViewIds))
        {
            ALICEVISION_LOG_ERROR("Failed to search matching views between the input SfMData and the reference SfMData");
            return EXIT_FAILURE;
        }

        if (commonViewIds.empty())
        {
            ALICEVISION_LOG_ERROR("No matching views found between the input SfMData and the reference SfMData");
            return EXIT_FAILURE;
        }

        ALICEVISION_LOG_INFO("Found " << commonViewIds.size() << " matching views.");

        if (transferPoses)
        {
            transferMatchingPoses(sfmData, sfmDataRef, commonViewIds);
        }

        if (transferIntrinsics)
        {
            transferMatchingIntrinsics(sfmData, sfmDataRef, commonViewIds);
        }

        if (transferLandmarks)
        {
            transferMatchingLandmarks(sfmData, sfmDataRef, commonViewIds);
        }

        if (transferSurveyPoints)
        {
            transferMatchingSurveyPoints(sfmData, sfmDataRef, commonViewIds);
        }
    }

    // Cleanup sfmData by removing unused elements
    sfmData.removeUnusedCameraPoses();
    sfmData.removeUnusedLandmarks();
    sfmData.removeUnusedIntrinsics();

    // Export the SfMData scene in the expected format
    ALICEVISION_LOG_INFO("Save into '" << outSfMDataFilename << "'");
    if (!sfmDataIO::save(sfmData, outSfMDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("An error occurred while trying to save '" << outSfMDataFilename << "'");
        return EXIT_FAILURE;
    }

    if (!outputViewsAndPosesFilepath.empty())
    {
        sfmDataIO::save(sfmData, outputViewsAndPosesFilepath, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS));
    }

    return EXIT_SUCCESS;
}
