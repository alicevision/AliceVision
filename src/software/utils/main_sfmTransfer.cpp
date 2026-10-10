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
#include <aliceVision/config.hpp>

#include <boost/program_options.hpp>

#include <string>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 2

using namespace aliceVision;
using namespace aliceVision::sfm;

namespace po = boost::program_options;

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

void updateMatchingViews(sfmData::SfMData& sfmData,
                         const sfmData::SfMData& sfmDataRef,
                         const std::vector<std::pair<IndexT, IndexT>>& commonViewIds)
{
    for (const auto& [destinationViewId, sourceViewId] : commonViewIds)
    {
        auto& destinationView = sfmData.getView(destinationViewId);
        const auto& sourceView = sfmDataRef.getView(sourceViewId);
        
        destinationView.setResectionId(sourceView.getResectionId());
    }
}

/**
 * @brief Transfer the pose of a reference view onto a destination view.
 *        If both views are rig-dependent, the rig structure is kept: the rig (frame) pose and the
 *        sub-pose of the source view are copied into the destination rig.
 *        Otherwise, the absolute pose of the source view is copied as an independent pose.
 * @param[in,out] sfmData The destination SfMData
 * @param[in] sfmDataRef The reference SfMData
 * @param[in,out] destinationView The destination view
 * @param[in] sourceView The matching reference view (with a defined pose)
 * @param[in,out] transferredRigPoses Destination rig pose ID -> source rig pose ID already transferred
 */
void transferPose(sfmData::SfMData& sfmData,
                  const sfmData::SfMData& sfmDataRef,
                  sfmData::View& destinationView,
                  const sfmData::View& sourceView,
                  std::map<IndexT, IndexT>& transferredRigPoses)
{
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
        sfmData.getPoses().assign(rigPoseId, sfmDataRef.getAbsolutePose(sourceView.getPoseId()));

        // Copy the sub-pose corresponding to this view only
        auto& subPoses = sfmData.getRigs().at(destinationView.getRigId()).getSubPoses();
        if (destinationView.getSubPoseId() >= subPoses.size())
        {
            subPoses.resize(destinationView.getSubPoseId() + 1);
        }
        subPoses[destinationView.getSubPoseId()] = sfmDataRef.getRigs().at(sourceView.getRigId()).getSubPose(sourceView.getSubPoseId());
        return;
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

bool transferPosesAndIntrinsics(sfmData::SfMData& sfmData,
                             sfmData::SfMData& sfmDataRef,
                             const std::vector<std::pair<IndexT, IndexT>>& commonViewIds,
                             bool transferPoses,
                             bool transferIntrinsics)
{
    // Destination rig pose ID -> source rig pose ID, to detect inconsistent frame matching
    std::map<IndexT, IndexT> transferredRigPoses;

    // Loop over matching views
    for (const auto& matchingViews : commonViewIds)
    {
        if (!sfmDataRef.isPoseAndIntrinsicDefined(matchingViews.second))
        {
            continue;
        }
        
        ALICEVISION_LOG_INFO("Processing view #" << matchingViews.first << " (matching with view #" << matchingViews.second << ")");
        sfmData::View & destinationView = sfmData.getView(matchingViews.first);
        const sfmData::View & sourceView = sfmDataRef.getView(matchingViews.second);

        if (transferPoses)
        {
            ALICEVISION_LOG_INFO("Transfer Pose");
            transferPose(sfmData, sfmDataRef, destinationView, sourceView, transferredRigPoses);
        }

        if (transferIntrinsics)
        {
            ALICEVISION_LOG_INFO("Transfer Intrinsics");
            const auto* sourceIntrinsic = sfmDataRef.getIntrinsicPtr(sourceView.getIntrinsicId());
            auto* destinationIntrinsic = sfmData.getIntrinsicPtr(destinationView.getIntrinsicId());
            destinationIntrinsic->assign(*sourceIntrinsic);
        }
    }

    return true;
}

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

    // Observations refer to the reference features: make their folders available.
    // Folders already present in the destination are skipped.
    sfmData.addFeaturesFolders(sfmDataRef.getFeaturesFolders());
    sfmData.addMatchesFolders(sfmDataRef.getMatchesFolders());
}

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
         "\t- from_intrinsicid: Update intrinsics with the same intrinsic ID. Only the intrinsics will be updated.\n")
        ("fileMatchingPattern", po::value<std::string>(&fileMatchingPattern)->default_value(fileMatchingPattern),
         "Matching pattern for the from_filepath method.\n")
        ("metadataMatchingList", po::value<std::vector<std::string>>(&metadataMatchingList)->multitoken()->default_value(metadataMatchingList),
         "List of metadata that should match to create the correspondences.\n")
        ("transferPoses", po::value<bool>(&transferPoses)->default_value(transferPoses),
         "Transfer poses.")
        ("transferIntrinsics", po::value<bool>(&transferIntrinsics)->default_value(transferIntrinsics),
         "Transfer intrinsics.")
        ("transferLandmarks", po::value<bool>(&transferLandmarks)->default_value(transferLandmarks),
         "Transfer landmarks.")
        ("transferSurveyPoints", po::value<bool>(&transferSurveyPoints)->default_value(transferSurveyPoints),
         "Transfer survey points.")
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
        // Copy matching intrinsics from the reference SfMData to the input SfMData
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
        // Copy matching poses from the reference SfMData to the input SfMData
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
            updateMatchingViews(sfmData, sfmDataRef, commonViewIds);
        }

        if (!transferPosesAndIntrinsics(sfmData,
                                        sfmDataRef,
                                        commonViewIds,
                                        transferPoses,
                                        transferIntrinsics))
        {
            ALICEVISION_LOG_ERROR("An error occurred while transferring data by matching views");
            return EXIT_FAILURE;
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
