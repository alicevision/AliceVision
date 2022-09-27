// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfm/pipeline/RigSequence.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/utils/alignment.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/config.hpp>

#include <boost/program_options.hpp>

#include <string>
#include <sstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::sfm;

namespace po = boost::program_options;


/**
* @brief Matching Views method enum
*/
enum class EMatchingMethod : unsigned char
{
    FROM_VIEWID = 0
    , FROM_FILEPATH
    , FROM_METADATA
    , FROM_INTRINSICID
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
    case EMatchingMethod::FROM_VIEWID:   return "from_viewid";
    case EMatchingMethod::FROM_FILEPATH: return "from_filepath";
    case EMatchingMethod::FROM_METADATA: return "from_metadata";
    case EMatchingMethod::FROM_INTRINSICID: return "from_intrinsicid";
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
    std::transform(method.begin(), method.end(), method.begin(), ::tolower); //tolower

    if (method == "from_viewid")   return EMatchingMethod::FROM_VIEWID;
    if (method == "from_filepath") return EMatchingMethod::FROM_FILEPATH;
    if (method == "from_metadata") return EMatchingMethod::FROM_METADATA;
    if (method == "from_intrinsicid") return EMatchingMethod::FROM_INTRINSICID;
    throw std::out_of_range("Invalid SfM alignment method : " + alignmentMethod);
}

inline std::istream& operator>>(std::istream& in, EMatchingMethod& alignment)
{
    std::string token;
    in >> token;
    alignment = EMatchingMethod_stringToEnum(token);
    return in;
}

inline std::ostream& operator<<(std::ostream& os, EMatchingMethod e)
{
    return os << EMatchingMethod_enumToString(e);
}


int aliceVision_main(int argc, char **argv)
{
    // command-line parameters

    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmDataFilename;
    std::string outSfMDataFilename;
    std::string sfmDataReferenceFilename;
    bool transferPoses = true;
    bool transferIntrinsics = true;
    EMatchingMethod matchingMethod = EMatchingMethod::FROM_VIEWID;
    std::string fileMatchingPattern;
    std::vector<std::string> metadataMatchingList = { "Make", "Model", "Exif:BodySerialNumber" , "Exif:LensSerialNumber" };
    std::string outputViewsAndPosesFilepath;

    po::options_description allParams("AliceVision sfmAlignment");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
             "SfMData file to align.")
        ("output,o", po::value<std::string>(&outSfMDataFilename)->required(),
            "Output SfMData scene.")
        ("reference,r", po::value<std::string>(&sfmDataReferenceFilename)->required(),
            "Path to the scene used as the reference coordinate system.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("method", po::value<EMatchingMethod>(&matchingMethod)->default_value(matchingMethod),
            "Matching Method:\n"
            "\t- from_viewid: Align cameras with same view Id\n"
            "\t- from_filepath: Align cameras with a filepath matching, using --fileMatchingPattern\n"
            "\t- from_metadata: Align cameras with matching metadata, using --metadataMatchingList\n")
        ("fileMatchingPattern", po::value<std::string>(&fileMatchingPattern)->default_value(fileMatchingPattern),
            "Matching pattern for the from_filepath method.\n")
        ("metadataMatchingList", po::value<std::vector<std::string>>(&metadataMatchingList)->multitoken()->default_value(metadataMatchingList),
            "List of metadata that should match to create the correspondences.\n")
        ("transferPoses", po::value<bool>(&transferPoses)->default_value(transferPoses),
            "Transfer poses.")
        ("transferIntrinsics", po::value<bool>(&transferIntrinsics)->default_value(transferIntrinsics),
            "Transfer intrinsics.")
        ("outputViewsAndPoses", po::value<std::string>(&outputViewsAndPosesFilepath),
            "Path of the output SfMData file.")
        ;

    po::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
            "verbosity level (fatal,  error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(optionalParams).add(logParams);

    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, allParams), vm);

        if (vm.count("help") || (argc == 1))
        {
            ALICEVISION_COUT(allParams);
            return EXIT_SUCCESS;
        }
        po::notify(vm);
    }
    catch (boost::program_options::required_option& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }
    catch (boost::program_options::error& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    // Load input scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read");
        return EXIT_FAILURE;
    }

    // Load reference scene
    sfmData::SfMData sfmDataRef;
    if (!sfmDataIO::Load(sfmDataRef, sfmDataReferenceFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The reference SfMData file '" << sfmDataReferenceFilename << "' cannot be read");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Search similarity transformation.");

    std::vector<std::pair<IndexT, IndexT>> commonViewIds;
    switch (matchingMethod)
    {
        case EMatchingMethod::FROM_VIEWID:
        {
            std::vector<IndexT> commonViewIdsTmp;
            getCommonViews(sfmData, sfmDataRef, commonViewIdsTmp);
            for (IndexT id : commonViewIdsTmp)
            {
                commonViewIds.push_back(std::make_pair(id, id));
            }
            break;
        }
        case EMatchingMethod::FROM_FILEPATH:
        {
            sfm::matchViewsByFilePattern(sfmData, sfmDataRef, fileMatchingPattern, commonViewIds);
            break;
        }
        case EMatchingMethod::FROM_METADATA:
        {
            sfm::matchViewsByMetadataMatching(sfmData, sfmDataRef, metadataMatchingList, commonViewIds);
            break;
        }
        case EMatchingMethod::FROM_INTRINSICID: 
        {
            break;
        }
    }
    ALICEVISION_LOG_DEBUG("Found " << commonViewIds.size() << " common views.");

    if (matchingMethod == EMatchingMethod::FROM_INTRINSICID) 
    {
        for (auto intrinsic : sfmData.getIntrinsics())
        {
            for (const auto intrinsicRef : sfmDataRef.getIntrinsics())
            {
                if (intrinsic.first == intrinsicRef.first)
                {
                    intrinsic.second->updateFromParams(intrinsicRef.second->getParams());
                    break;
                }
            }
        }
    }
    else if (!transferPoses && !transferIntrinsics)
    {
        ALICEVISION_LOG_ERROR("Nothing to do.");
    }
    else
    {
        if (commonViewIds.empty())
        {
            ALICEVISION_LOG_ERROR("Failed to find matching Views between the 2 SfmData.");
            return EXIT_FAILURE;
        }
        for (const auto& matchingViews: commonViewIds)
        {
            if(sfmDataRef.isPoseAndIntrinsicDefined(matchingViews.second))
            {
                // Missing pose in sfmData and valid pose in sfmDataRef,
                // so we can transfer the pose.

                auto& viewA = sfmData.getView(matchingViews.first);
                const auto& viewB = sfmDataRef.getView(matchingViews.second);

                if (transferPoses)
                {
                    ALICEVISION_LOG_TRACE("Transfer pose (pose id: " << viewA.getPoseId() << " <- " << viewB.getPoseId() << ", " << viewA.getImagePath() << " <- " << viewB.getImagePath() << ").");

                    if (viewA.isPartOfRig() && viewB.isPartOfRig())
                    {
                        ALICEVISION_LOG_TRACE("Transfer rig (rig id: " << viewA.getRigId() << " <- " << viewB.getRigId() << ", " << viewA.getImagePath() << " <- " << viewB.getImagePath() << ").");

                        if (!viewB.isPoseIndependant())
                        {
                            if (viewA.isPoseIndependant())
                            {
                                IndexT rigPoseId = sfm::getRigPoseId(viewA.getRigId(), viewA.getFrameId());
                                viewA.setPoseId(rigPoseId);
                                viewA.setIndependantPose(false);
                            }
                            else
                            {
                                if (viewA.getPoseId() == viewA.getPoseId())
                                    throw std::runtime_error("Invalid RigId/PoseId (in rig) for view: " + viewA.getImagePath());
                            }
                        }
                        else
                        {
                            if (!viewA.isPoseIndependant())
                            {
                                viewA.setPoseId(viewA.getViewId());
                                viewA.setIndependantPose(viewB.isPoseIndependant());
                            }
                            else
                            {
                                if (viewA.getPoseId() != viewA.getPoseId())
                                    throw std::runtime_error("Invalid RigId/PoseId (out of rig) for view: " + viewA.getImagePath());
                            }
                        }
                        // copy the pose of the rig or the independant pose
                        sfmData.getPoses()[viewA.getPoseId()] = sfmDataRef.getPoses().at(viewB.getPoseId());

                        // warning: we copy the full rig (and not only the subpose corresponding to the view).
                        sfmData.getRigs()[viewA.getRigId()] = sfmDataRef.getRigs()[viewB.getRigId()];
                    }
                    else
                    {
                        if (viewA.isPartOfRig() && !viewA.isPoseIndependant())
                        {
                            viewA.setPoseId(viewA.getViewId());
                            viewA.setIndependantPose(true);
                        }
                        sfmData.getPoses()[viewA.getPoseId()] = sfmDataRef.getPose(viewB);
                    }
                }
                if (transferIntrinsics)
                {
                    ALICEVISION_LOG_TRACE("Transfer intrinsics (intrinsic id: " << viewA.getIntrinsicId() << " <- " << viewB.getIntrinsicId() << ", " << viewA.getImagePath() << " <- " << viewB.getImagePath() << ").");
                    sfmData.getIntrinsicPtr(viewA.getIntrinsicId())->assign(*sfmDataRef.getIntrinsicPtr(viewB.getIntrinsicId()));
                }
            }
        }
    }

    // Pose Id to remove
    {
        std::set<IndexT> usedPoseIds;
        for (auto viewIt : sfmData.getViews())
        {
            usedPoseIds.insert(viewIt.second->getPoseId());
        }
        std::set<IndexT> poseIdsToRemove;
        for (auto poseIt : sfmData.getPoses())
        {
            if (usedPoseIds.find(poseIt.first) == usedPoseIds.end())
            {
                poseIdsToRemove.insert(poseIt.first);
            }
        }
        for (auto r : poseIdsToRemove)
            sfmData.getPoses().erase(r);
    }

    ALICEVISION_LOG_INFO("Save into '" << outSfMDataFilename << "'");
    // Export the SfMData scene in the expected format
    if (!sfmDataIO::Save(sfmData, outSfMDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("An error occurred while trying to save '" << outSfMDataFilename << "'");
        return EXIT_FAILURE;
    }

    if(!outputViewsAndPosesFilepath.empty())
    {
        sfmDataIO::Save(sfmData, outputViewsAndPosesFilepath,
                        sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::EXTRINSICS | sfmDataIO::INTRINSICS));
    }

    return EXIT_SUCCESS;
}
