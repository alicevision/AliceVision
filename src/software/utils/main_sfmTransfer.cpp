// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/utils/alignment.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/program_options.hpp>

#include <map>
#include <set>
#include <string>

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

/**
 * @brief Transfer the poses of the reference views onto their matching destination views.
 *        If both views are rig-dependent, the rig structure is kept: the rig (frame) pose and the
 *        sub-pose of the source view are copied into the destination rig.
 *        Otherwise, the absolute pose of the source view is copied as an independent pose.
 *
 *        Pairs whose reference view has no valid pose and intrinsic are skipped.
 *        The resection ID of the reference view is copied onto the destination view.
 *
 *        Rig case (reference view rig-dependent and destination rig existing):
 *        - the destination view is attached to the rig pose getRigPoseId(rigId, frameId), and made rig-dependent;
 *        - this rig pose is overwritten with the reference rig (frame) pose. If several reference frames
 *          target the same destination frame, a warning is emitted and the last one is kept;
 *        - only the sub-pose of this destination view is overwritten (sub-poses are resized if needed),
 *          the other sub-poses of the destination rig are kept.
 *
 *        Independent case: the reference absolute pose (rig pose composed with sub-pose if rig-dependent)
 *        is written. A rig-dependent destination view is detached from its rig: its pose ID becomes its
 *        view ID. Any existing pose with this ID is overwritten.
 *
 *        Destination poses no longer used are removed by the final cleanup.
 * @param[in,out] sfmData The destination SfMData
 * @param[in] sfmDataRef The reference SfMData
 * @param[in] commonViewIds Matching (destination view ID, reference view ID) pairs
 */
void transferMatchingPoses(sfmData::SfMData& sfmData,
                           const sfmData::SfMData& sfmDataRef,
                           const std::vector<std::pair<IndexT, IndexT>>& commonViewIds)
{
    ALICEVISION_LOG_INFO("Transfer Poses");

    // Destination rig pose ID -> source rig pose ID, to detect inconsistent frame matching
    std::map<IndexT, IndexT> transferredRigPoses;

    for (const auto& [destinationViewId, sourceViewId] : commonViewIds)
    {
        if (!sfmDataRef.isPoseAndIntrinsicDefined(sourceViewId))
        {
            continue;
        }

        sfmData::View& destinationView = sfmData.getView(destinationViewId);
        const sfmData::View& sourceView = sfmDataRef.getView(sourceViewId);
        destinationView.setResectionId(sourceView.getResectionId());

        const bool sourceInRig = sourceView.isPartOfRig() && !sourceView.isPoseIndependant();
        const bool destinationRigExists = destinationView.isPartOfRig() && sfmData.getRigs().count(destinationView.getRigId()) > 0;

        if (sourceInRig && destinationRigExists)
        {
            // Attach the destination view to its rig frame pose
            const IndexT rigPoseId = sfmData::getRigPoseId(destinationView.getRigId(), destinationView.getFrameId());
            destinationView.setPoseId(rigPoseId);
            destinationView.setIndependantPose(false);

            // Copy the rig (frame) pose
            const auto [it, inserted] = transferredRigPoses.emplace(rigPoseId, sourceView.getPoseId());
            if (!inserted && it->second != sourceView.getPoseId())
            {
                ALICEVISION_LOG_WARNING("Rig pose " << rigPoseId << " receives poses from different reference frames ("
                                                    << it->second << " and " << sourceView.getPoseId() << "), keeping the last one. "
                                                    << "View: " << destinationView.getImage().getImagePath());
            }
            // Track the source of the pose currently assigned
            it->second = sourceView.getPoseId();
            sfmData.getPoses().assign(rigPoseId, sfmDataRef.getAbsolutePose(sourceView.getPoseId()));

            // Copy the sub-pose corresponding to this view only
            auto& subPoses = sfmData.getRigs().at(destinationView.getRigId()).getSubPoses();
            if (destinationView.getSubPoseId() >= subPoses.size())
            {
                subPoses.resize(destinationView.getSubPoseId() + 1);
            }
            subPoses[destinationView.getSubPoseId()] = sfmDataRef.getRigs().at(sourceView.getRigId()).getSubPose(sourceView.getSubPoseId());
            continue;
        }

        // Independent pose: the destination view gets its own pose, even if part of a rig
        if (destinationView.isPartOfRig() && !destinationView.isPoseIndependant())
        {
            destinationView.setPoseId(destinationView.getViewId());
            destinationView.setIndependantPose(true);
        }

        // getPose() composes rig pose and sub-pose if the source view is rig-dependent
        sfmData.setPose(destinationView, sfmDataRef.getPose(sourceView));
    }
}

/**
 * @brief Transfer the intrinsics of the reference views onto their matching destination views.
 *
 *        For each destination intrinsic, the first reference intrinsic that matches it is copied into it.
 *        If views of this destination intrinsic match another reference intrinsic, they are moved
 *        onto a new destination intrinsic holding a copy of it.
 *        Destination intrinsics are never merged, even if they match the same reference intrinsic.
 *
 *        Pairs whose reference view has no valid pose and intrinsic are skipped.
 *        As the first copy overwrites the destination intrinsic in place, unmatched destination views
 *        using this intrinsic also get the reference values.
 *        A destination view whose intrinsic ID does not exist gets a new intrinsic.
 *        New intrinsic IDs are allocated after the highest existing ID.
 *        Intrinsics are cloned, so the camera model of the destination may change.
 *
 * @param[in,out] sfmData The destination SfMData
 * @param[in] sfmDataRef The reference SfMData
 * @param[in] commonViewIds Matching (destination view ID, reference view ID) pairs
 */
void transferMatchingIntrinsics(sfmData::SfMData& sfmData,
                                const sfmData::SfMData& sfmDataRef,
                                const std::vector<std::pair<IndexT, IndexT>>& commonViewIds)
{
    ALICEVISION_LOG_INFO("Transfer Intrinsics");

    auto& intrinsics = sfmData.getIntrinsics();

    // (original destination intrinsic ID, reference intrinsic ID) -> destination intrinsic ID now holding the copy
    std::map<std::pair<IndexT, IndexT>, IndexT> copies;
    // Original destination intrinsics already overwritten by a copy
    std::set<IndexT> overwritten;

    for (const auto& [destinationViewId, sourceViewId] : commonViewIds)
    {
        if (!sfmDataRef.isPoseAndIntrinsicDefined(sourceViewId))
        {
            continue;
        }

        sfmData::View& destinationView = sfmData.getView(destinationViewId);
        const IndexT sourceIntrinsicId = sfmDataRef.getView(sourceViewId).getIntrinsicId();
        const IndexT destinationIntrinsicId = destinationView.getIntrinsicId();

        auto [it, isNewPair] = copies.emplace(std::make_pair(destinationIntrinsicId, sourceIntrinsicId), UndefinedIndexT);
        if (isNewPair)
        {
            // Overwrite the destination intrinsic the first time, otherwise create a new one
            if (intrinsics.count(destinationIntrinsicId) && !overwritten.count(destinationIntrinsicId))
            {
                it->second = destinationIntrinsicId;
                overwritten.insert(destinationIntrinsicId);
            }
            else
            {
                it->second = intrinsics.empty() ? 0 : intrinsics.rbegin()->first + 1;
            }

            // Clone instead of assign, as the camera model may differ
            intrinsics.insert_or_assign(it->second, std::shared_ptr<camera::IntrinsicBase>(sfmDataRef.getIntrinsics().at(sourceIntrinsicId)->clone()));
        }

        destinationView.setIntrinsicId(it->second);
    }
}

/**
 * @brief Replace the destination landmarks by the reference landmarks seen by matching views.
 *        If the reference SfMData has no landmarks, the destination landmarks are kept untouched.
 *        Otherwise, all destination landmarks are discarded. Each reference landmark is copied with its ID,
 *        its observations being re-indexed onto the destination view IDs. Observations in unmatched reference
 *        views are dropped, as well as landmarks without any remaining observation.
 *        The reference views are not required to be reconstructed.
 * @param[in,out] sfmData The destination SfMData
 * @param[in] sfmDataRef The reference SfMData
 * @param[in] commonViewIds Matching (destination view ID, reference view ID) pairs
 */
void transferMatchingLandmarks(sfmData::SfMData& sfmData,
                               const sfmData::SfMData& sfmDataRef,
                               const std::vector<std::pair<IndexT, IndexT>>& commonViewIds)
{
    ALICEVISION_LOG_INFO("Transfer Landmarks");

    const auto& refLandmarks = sfmDataRef.getLandmarks();
    if (refLandmarks.empty())
    {
        return;
    }

    // Map reference view IDs to destination view IDs.
    std::map<IndexT, IndexT> commonViewsMap;
    for (const auto& viewPair : commonViewIds)
    {
        commonViewsMap.emplace(viewPair.second, viewPair.first);
    }

    // Copy landmarks from the reference SfMData to the destination SfMData, 
    // only keeping observations that have matching views.

    sfmData::Landmarks newLandmarks;
    for (const auto& landIt : refLandmarks)
    {
        sfmData::Landmark newLandmark = landIt.second;
        newLandmark.getObservations().clear();

        for (const auto& obsIt : landIt.second.getObservations())
        {
            const auto matchingView = commonViewsMap.find(obsIt.first);
            if (matchingView != commonViewsMap.end())
            {
                newLandmark.getObservations().emplace(matchingView->second, obsIt.second);
            }
        }

        if (!newLandmark.getObservations().empty())
        {
            newLandmarks.emplace(landIt.first, newLandmark);
        }
    }

    sfmData.getLandmarks() = newLandmarks;


}

/**
 * @brief Transfer the survey points of the reference views onto their matching destination views.
 *        Survey points are stored per view: for each matching pair where the reference view has survey points,
 *        they replace the survey points of the destination view. Other destination views keep their survey points.
 * @param[in,out] sfmData The destination SfMData
 * @param[in] sfmDataRef The reference SfMData
 * @param[in] commonViewIds Matching (destination view ID, reference view ID) pairs
 */
void transferMatchingSurveyPoints(sfmData::SfMData& sfmData,
                                  const sfmData::SfMData& sfmDataRef,
                                  const std::vector<std::pair<IndexT, IndexT>>& commonViewIds)
{
    ALICEVISION_LOG_INFO("Transfer Survey Points");

    const auto& refSurveyPoints = sfmDataRef.getSurveyPoints();

    // Survey points are stored per view: copy them from each reference view onto its matching destination view
    for (const auto& [destinationViewId, sourceViewId] : commonViewIds)
    {
        const auto it = refSurveyPoints.find(sourceViewId);
        if (it != refSurveyPoints.end())
        {
            sfmData.getSurveyPoints()[destinationViewId] = it->second;
        }
    }
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
