// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/types.hpp>
#include <aliceVision/config.hpp>

#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfm/pipeline/regionsIO.hpp>

#include <aliceVision/track/TracksBuilder.hpp>
#include <aliceVision/track/tracksUtils.hpp>
#include <aliceVision/track/trackIO.hpp>
#include <aliceVision/track/TracksHandler.hpp>

#include <aliceVision/camera/Pinhole.hpp>

#include <aliceVision/robustEstimation/NACRansac.hpp>
#include <aliceVision/multiview/triangulation/triangulationDLT.hpp>
#include <aliceVision/multiview/relativePose/RelativeSphericalKernel.hpp>
#include <aliceVision/multiview/relativePose/RotationSphericalKernel.hpp>

#include <aliceVision/multiview/essential.hpp>
#include <aliceVision/geometry/lie.hpp>
#include <aliceVision/sfm/utils/statistics.hpp>

#include <boost/program_options.hpp>

#include <aliceVision/sfm/pipeline/relativePoses.hpp>

#include <cstdlib>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;


/**
 * @brief Estimate the relative essential matrix between two cameras
 * @param E the output Essential matrix
 * @param vecInliers the input list of inliers (set of indices in the coordinates vectors)
 * @param cam1 the first camera intrinsic object
 * @param cam2 the second camera intrinsic object
 * @param x1 the observed points coordinates in the first camera
 * @param x2 the observed points coordinates in the second camera
 * @param randomNumberGenerator a random number generator object shared among objects
 * @param maxIterationsCount how many iterations are allowed during ransac
 * @param minInliers what is the minimal number of inliers required to consider the estimation successful
 * @return true if estimation succeeded
*/
bool robustEssential(Mat3& E,
                     std::vector<size_t>& vecInliers,
                     const camera::IntrinsicBase & cam1,
                     const camera::IntrinsicBase & cam2,
                     const std::vector<Vec2>& x1,
                     const std::vector<Vec2>& x2,
                     std::mt19937& randomNumberGenerator,
                     const size_t maxIterationCount,
                     const size_t minInliers)
{
    multiview::relativePose::RelativeSphericalKernel kernel(cam1, cam2, x1, x2);

    robustEstimation::Mat3Model model;
    vecInliers.clear();

    // robustly estimation of the Essential matrix and its precision
    const std::pair<double, double> acRansacOut =
      robustEstimation::NACRANSAC(kernel, randomNumberGenerator, vecInliers, maxIterationCount, &model);

    if (vecInliers.size() < minInliers)
    {
        return false;
    }

    E = model.getMatrix();

    return true;
}

/**
 * @brief Estimate the relative rortation matrix between two cameras
 * @param R the output Rotation matrix
 * @param vecInliers the input list of inliers (set of indices in the coordinates vectors)
 * @param cam1 the first camera intrinsic object
 * @param cam2 the second camera intrinsic object
 * @param x1 the observed points coordinates in the first camera
 * @param x2 the observed points coordinates in the second camera
 * @param randomNumberGenerator a random number generator object shared among objects
 * @param maxIterationsCount how many iterations are allowed during ransac
 * @param minInliers what is the minimal number of inliers required to consider the estimation successful
 * @return true if estimation succeeded
*/
bool robustRotation(Mat3& R,
                    std::vector<size_t>& vecInliers,
                     const camera::IntrinsicBase & cam1,
                     const camera::IntrinsicBase & cam2,
                     const std::vector<Vec2>& x1,
                     const std::vector<Vec2>& x2,
                     std::mt19937& randomNumberGenerator,
                     const size_t maxIterationCount,
                     const size_t minInliers)
{
    multiview::relativePose::RotationSphericalKernel kernel(cam1, cam2, x1, x2);

    robustEstimation::Mat3Model model;
    vecInliers.clear();

    // robustly estimation of the Essential matrix and its precision
    const std::pair<double, double> acRansacOut =
      robustEstimation::NACRANSAC(kernel, randomNumberGenerator, vecInliers, maxIterationCount, &model);

    if (vecInliers.size() < minInliers)
    {
        return false;
    }

    R = model.getMatrix();

    return true;
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string tracksFilename;
    std::string outputDirectory;
    int rangeStart = -1;
    int rangeSize = 1;
    const size_t minInliers = 35;
    bool enforcePureRotation = false;

    // user optional parameters
    std::string describerTypesName = feature::EImageDescriberType_enumToString(feature::EImageDescriberType::SIFT);

    int randomSeed = std::mt19937::default_seed;

    // clang-format off
    po::options_description requiredParams("Required parameters");
        requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")
        ("tracksFilename,t", po::value<std::string>(&tracksFilename)->required(), "Tracks file.")
        ("output,o", po::value<std::string>(&outputDirectory)->required(),"Path to the output directory.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("enforcePureRotation,e", po::value<bool>(&enforcePureRotation)->default_value(enforcePureRotation), "Enforce pure rotation in estimation.")
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart), "Range image index start.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize), "Range size.");
    // clang-format on

    CmdLine cmdline("AliceVision relativePoseEstimating");

    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();
    omp_set_num_threads(hwc.getMaxThreads());

    std::mt19937 randomNumberGenerator(randomSeed);

    // load input SfMData scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }

    // Define range to compute
    if(rangeStart != -1)
    {
        if(rangeStart < 0 || rangeSize < 0)
        {
            ALICEVISION_LOG_ERROR("Range is incorrect");
            return EXIT_FAILURE;
        }

        if (rangeStart > sfmData.getViews().size())
        {
            ALICEVISION_LOG_INFO("Empty range to compute");
            return EXIT_SUCCESS;
        }

        if(rangeStart + rangeSize > sfmData.getViews().size())
        {
            rangeSize = sfmData.getViews().size() - rangeStart;
        }
    }
    else
    {
        rangeStart = 0;
        rangeSize = sfmData.getViews().size();
    }
    ALICEVISION_LOG_DEBUG("Range to compute: rangeStart=" << rangeStart << ", rangeSize=" << rangeSize);


    // Load tracks
    ALICEVISION_LOG_INFO("Load tracks");
    track::TracksHandler tracksHandler;
    if (!tracksHandler.load(tracksFilename, sfmData.getViewsKeys()))
    {
        ALICEVISION_LOG_ERROR("The input tracks file '" + tracksFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }
    
    //Compute covisibility for tracks
    //This will get the list of pair of views which observe common features
    ALICEVISION_LOG_INFO("Compute co-visibility");
    std::map<Pair, unsigned int> covisibility;
    track::computeCovisibility(covisibility, tracksHandler.getAllTracks());

    ALICEVISION_LOG_INFO("Process co-visibility");
    std::stringstream ss;
    ss << outputDirectory << "/pairs_" << rangeStart << ".json";
    std::ofstream of(ss.str());

    //Output container
    std::vector<sfm::ReconstructedPair> reconstructedPairs;

    double ratioChunk = double(covisibility.size()) / double(sfmData.getViews().size());
    int chunkStart = int(double(rangeStart) * ratioChunk);
    int chunkEnd = int(double(rangeStart + rangeSize) * ratioChunk);

    // For each covisible pair
#pragma omp parallel for
    for (int posPairs = chunkStart; posPairs < chunkEnd; posPairs++)
    {
        auto iterPairs = covisibility.begin();
        std::advance(iterPairs, posPairs);

        // Retrieve pair information
        IndexT refImage = iterPairs->first.first;
        IndexT nextImage = iterPairs->first.second;

        const sfmData::View& refView = sfmData.getView(refImage);
        const sfmData::View& nextView = sfmData.getView(nextImage);

        std::shared_ptr<camera::IntrinsicBase> refIntrinsics = sfmData.getIntrinsicSharedPtr(refView.getIntrinsicId());
        std::shared_ptr<camera::IntrinsicBase> nextIntrinsics = sfmData.getIntrinsicSharedPtr(nextView.getIntrinsicId());
        

        aliceVision::track::TracksMap mapTracksCommon;
        track::getCommonTracksInImagesFast({refImage, nextImage}, tracksHandler.getAllTracks(), tracksHandler.getTracksPerView(), mapTracksCommon);

        // Build features coordinates matrices
        const std::size_t n = mapTracksCommon.size();
        std::vector<Eigen::Vector2d> refpts, nextpts;
        for (const auto& commonItem : mapTracksCommon)
        {
            const track::Track & track = commonItem.second;
            refpts.push_back(track.featPerView.at(refImage).coords);
            nextpts.push_back(track.featPerView.at(nextImage).coords);
        }

        std::vector<size_t> vecInliers;
        sfm::ReconstructedPair reconstructed;

        if (enforcePureRotation)
        {
            // Try to fit an essential matrix (we assume we are approx. calibrated)
            Mat3 R;
            const bool relativeSuccess = robustRotation(R, 
                                                        vecInliers, 
                                                        *refIntrinsics,
                                                        *nextIntrinsics, 
                                                        refpts, 
                                                        nextpts, 
                                                        randomNumberGenerator, 
                                                        1024, 
                                                        minInliers);
            if (!relativeSuccess)
            {
                continue;
            }

            reconstructed.reference = refImage;
            reconstructed.next = nextImage;
            reconstructed.R = R;
            reconstructed.t.fill(0);
        }
        else
        {
            // Try to fit an essential matrix (we assume we are approx. calibrated)
            Mat3 E;
            std::vector<size_t> inliers;
            const bool essentialSuccess = robustEssential(E,
                                                          inliers,
                                                          *refIntrinsics,
                                                          *nextIntrinsics,
                                                          refpts,
                                                          nextpts,
                                                          randomNumberGenerator,
                                                          1024,
                                                          minInliers);
            if (!essentialSuccess)
            {
                continue;
            }

            std::vector<Vec3> structure;
            reconstructed.reference = refImage;
            reconstructed.next = nextImage;

            Mat4 T;
            if (!estimateTransformStructureFromEssential(T, structure, vecInliers, E, inliers, 
                                                      *refIntrinsics, *nextIntrinsics, 
                                                      refpts, nextpts))
            {
                continue;
            }

            reconstructed.R = T.block<3, 3>(0, 0);
            reconstructed.t = T.block<3, 1>(0, 3);
        }
        
        // Extract inliers
        std::vector<Vec2> refPtsValid, nextPtsValid;
        for (auto id : vecInliers)
        {
            refPtsValid.push_back(refpts[id]);
            nextPtsValid.push_back(nextpts[id]);
        }

        // Compute matched points coverage of image
        const double areaScore = sfm::computeAreaScore(refPtsValid, nextPtsValid, refIntrinsics->w(), refIntrinsics->h(), nextIntrinsics->w(), nextIntrinsics->h());

        // Compute ratio of matched points
        const double iunion = n;
        const double iinter = vecInliers.size();
        const double score = iinter / iunion;
        reconstructed.score = 0.5 * score + 0.5 * areaScore;

// Buffered output to avoid lo
#pragma omp critical
        {
            reconstructedPairs.push_back(reconstructed);

            if (reconstructedPairs.size() > 1000)
            {
                boost::json::value jv = boost::json::value_from(reconstructedPairs);
                of << boost::json::serialize(jv);
                reconstructedPairs.clear();
            }
        }
    }

    // Serialize last pairs
    {
        boost::json::value jv = boost::json::value_from(reconstructedPairs);
        of << boost::json::serialize(jv);
    }

    of.close();

    return EXIT_SUCCESS;
}
