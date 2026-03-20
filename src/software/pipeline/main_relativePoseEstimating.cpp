// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/types.hpp>
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/config.hpp>

#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Parallelization.hpp>
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

#include <aliceVision/matchingImageCollection/ImagePairListIO.hpp>

#include <aliceVision/multiview/essential.hpp>
#include <aliceVision/geometry/lie.hpp>
#include <aliceVision/sfm/utils/statistics.hpp>

#include <boost/program_options.hpp>

#include <aliceVision/sfm/pipeline/relativePoses.hpp>

#include <cstdlib>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 3
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;

namespace po = boost::program_options;


/**
 * @brief Estimate the relative essential matrix between two cameras
 * @param E the output Essential matrix
 * @param vecInliers the input list of inliers (set of indices in the coordinates vectors)
 * @param errorMax the allowed error for an inlier
 * @param cam1 the first camera intrinsic object
 * @param cam2 the second camera intrinsic object
 * @param x1 the observed points coordinates in the first camera
 * @param x2 the observed points coordinates in the second camera
 * @param randomNumberGenerator a random number generator object shared among objects
 * @param maxIterationsCount how many iterations are allowed during ransac
 * @param minInliers what is the minimal number of inliers required to consider the estimation successful
 * @param distanceThreshold minimal distance allowed for epipolar line to point distance in pixels
 * @return true if estimation succeeded
*/
bool robustEssential(Mat3& E,
                     std::vector<size_t>& vecInliers,
                     double & errorMax,
                     const camera::IntrinsicBase & cam1,
                     const camera::IntrinsicBase & cam2,
                     const std::vector<Vec2>& x1,
                     const std::vector<Vec2>& x2,
                     std::mt19937& randomNumberGenerator,
                     const size_t maxIterationCount,
                     const size_t minInliers,
                     const double distanceThreshold)
{
    multiview::relativePose::RelativeSphericalKernel kernel(cam1, cam2, x1, x2);

    double threshold = std::numeric_limits<double>::infinity();
    if (distanceThreshold > 0.0)
    {
        //From pixel distance to angular error
        double angularError1 = cam1.getHorizontalFov() * distanceThreshold / cam1.w();
        double angularError2 = cam2.getHorizontalFov() * distanceThreshold / cam2.w();
        threshold = std::max(angularError1, angularError2);
    }

    robustEstimation::Mat3Model model;
    vecInliers.clear();

    // robustly estimation of the Essential matrix and its precision
    const std::pair<double, double> acRansacOut =
      robustEstimation::NACRANSAC(kernel, 
                                randomNumberGenerator, 
                                vecInliers, 
                                maxIterationCount, 
                                &model, 
                                threshold);

    if (vecInliers.size() < minInliers)
    {
        return false;
    }

    E = model.getMatrix();

    errorMax = acRansacOut.first;

    return true;
}

/**
 * @brief Estimate the relative rortation matrix between two cameras
 * @param R the output Rotation matrix
 * @param vecInliers the input list of inliers (set of indices in the coordinates vectors)
 * @param errorMax the allowed error for an inlier
 * @param cam1 the first camera intrinsic object
 * @param cam2 the second camera intrinsic object
 * @param x1 the observed points coordinates in the first camera
 * @param x2 the observed points coordinates in the second camera
 * @param randomNumberGenerator a random number generator object shared among objects
 * @param maxIterationsCount how many iterations are allowed during ransac
 * @param minInliers what is the minimal number of inliers required to consider the estimation successful
 * @param distanceThreshold minimal distance allowed to reprojection
 * @return true if estimation succeeded
*/
bool robustRotation(Mat3& R,
                    std::vector<size_t>& vecInliers,
                    double & errorMax,
                     const camera::IntrinsicBase & cam1,
                     const camera::IntrinsicBase & cam2,
                     const std::vector<Vec2>& x1,
                     const std::vector<Vec2>& x2,
                     std::mt19937& randomNumberGenerator,
                     const size_t maxIterationCount,
                     const size_t minInliers,
                     const double distanceThreshold)
{
    multiview::relativePose::RotationSphericalKernel kernel(cam1, cam2, x1, x2);

    double threshold = std::numeric_limits<double>::infinity();
    if (distanceThreshold > 0.0)
    {
        //From pixel distance to angular error
        double angularError1 = cam1.getHorizontalFov() * distanceThreshold / cam1.w();
        double angularError2 = cam2.getHorizontalFov() * distanceThreshold / cam2.w();
        threshold = std::max(angularError1, angularError2);
    }

    robustEstimation::Mat3Model model;
    vecInliers.clear();

    // robustly estimation of the Essential matrix and its precision
    const std::pair<double, double> acRansacOut =
      robustEstimation::NACRANSAC(kernel,
                                randomNumberGenerator, 
                                vecInliers, 
                                maxIterationCount, 
                                &model, 
                                threshold);

    if (vecInliers.size() < minInliers)
    {
        return false;
    }

    R = model.getMatrix();

    errorMax = acRansacOut.first;

    return true;
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string tracksFilename;
    std::string outputDirectory;
    int rangeIteration = 0;
    int rangeBlocksCount = 1;
    size_t minInliers = 35;
    bool enforcePureRotation = false;
    size_t countIterations = 1024;
    std::vector<std::string> predefinedPairList;
    double distanceThreshold = 4.0;

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
        ("countIterations", po::value<size_t>(&countIterations)->default_value(countIterations), "Maximal number of iterations.")
        ("minInliers", po::value<size_t>(&minInliers)->default_value(minInliers), "Minimal number of inliers for a valid ransac.")
        ("distanceThreshold", po::value<double>(&distanceThreshold)->default_value(distanceThreshold), "Threshold on geometric distance (epipolar distance or reprojection distance for pure rotation)")
        ("imagePairsList,l", po::value<std::vector<std::string>>(&predefinedPairList)->multitoken(),
         "Path(s) to one or more files which contain the list of image pairs to match.")
        ("rangeIteration", po::value<int>(&rangeIteration)->default_value(rangeIteration), "Chunk id.")
        ("rangeBlocksCount", po::value<int>(&rangeBlocksCount)->default_value(rangeBlocksCount), "Chunk count.");
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

    // Generate one number generator per thread to enable repetability
    // Without thread concurrency
    std::vector<std::mt19937> randomNumberGenerators;
    for (int i = 0; i < omp_get_max_threads(); i++)
    {
        randomNumberGenerators.emplace_back(randomSeed);
    }

    // load input SfMData scene
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmDataFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }


    // Load tracks
    ALICEVISION_LOG_INFO("Load tracks from " << tracksFilename << ".");
    track::TracksHandler tracksHandler;
    if (!tracksHandler.load(tracksFilename, sfmData.getViewsKeys()))
    {
        ALICEVISION_LOG_ERROR("The input tracks file '" + tracksFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }   

    ALICEVISION_LOG_INFO("Compute co-visibility");
    std::map<Pair, unsigned int> covisibility;

    
        
    if (predefinedPairList.empty())
    {
        //Compute covisibility for tracks
        //This will get the list of pair of views which observe common features
        ALICEVISION_LOG_INFO("Automatically select pairs.");
        track::computeCovisibility(covisibility, tracksHandler.getAllTracks());
    }
    else
    {
        //Load pairs from file
        for (const std::string& imagePairsFile : predefinedPairList)
        {
            PairSet pairs;
                
            ALICEVISION_LOG_INFO("Load pair list from file: " << imagePairsFile);
            if (!matchingImageCollection::loadPairsFromFile(imagePairsFile, pairs))
            {
                return EXIT_FAILURE;
            }

            //Reformat to the same structure than track::computeCovisibility
            for (const auto & pair : pairs)
            {
                covisibility[pair] = 1;

                //Make sure we test symmetrically too
                Pair other;
                other.first = pair.second;
                other.second = pair.first;
                covisibility[other] = 1;
            }
        }
    }

    int chunkStart, chunkEnd;
    if (!rangeComputation(chunkStart, chunkEnd, rangeIteration, rangeBlocksCount, covisibility.size()))
    {
        ALICEVISION_LOG_INFO("Nothing to compute in this chunk");
    }

    ALICEVISION_LOG_INFO("A total of " << covisibility.size() << " pairs has to be processed.");
    ALICEVISION_LOG_INFO("Current chunk will analyze pairs from " << chunkStart << " to " << chunkEnd << ".");

    //Output container
    std::vector<sfm::ReconstructedPair> reconstructedPairs;

    ALICEVISION_LOG_INFO("Process co-visibility");
    std::stringstream ss;
    ss << outputDirectory << "/pairs_" << rangeIteration << ".json";
    std::ofstream of(ss.str());

    // For each covisible pair
#pragma omp parallel for //schedule(dynamic)
    for (int posPairs = chunkStart; posPairs < chunkEnd; posPairs++)
    {
        auto iterPairs = covisibility.begin();
        std::advance(iterPairs, posPairs);

        std::mt19937 & randomNumberGenerator = randomNumberGenerators[omp_get_thread_num()];

        // Retrieve pair information
        IndexT refImage = iterPairs->first.first;
        IndexT nextImage = iterPairs->first.second;

        const sfmData::View& refView = sfmData.getView(refImage);
        const sfmData::View& nextView = sfmData.getView(nextImage);

        std::shared_ptr<camera::IntrinsicBase> refIntrinsics = sfmData.getIntrinsicSharedPtr(refView.getIntrinsicId());
        std::shared_ptr<camera::IntrinsicBase> nextIntrinsics = sfmData.getIntrinsicSharedPtr(nextView.getIntrinsicId());
        

        aliceVision::track::TracksMap mapTracksCommon;
        track::getCommonTracksInImagesFast({refImage, nextImage}, tracksHandler.getAllTracks(), tracksHandler.getTracksPerView(), mapTracksCommon);

        if (mapTracksCommon.size() == 0)
        {
            continue;
        }

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
        double errorMax = 0.0;

        if (enforcePureRotation)
        {
            // Try to fit an essential matrix (we assume we are approx. calibrated)
            Mat3 R;
            const bool relativeSuccess = robustRotation(R, 
                                                        vecInliers, 
                                                        errorMax,
                                                        *refIntrinsics,
                                                        *nextIntrinsics, 
                                                        refpts, 
                                                        nextpts, 
                                                        randomNumberGenerator, 
                                                        countIterations, 
                                                        minInliers,
                                                        distanceThreshold);
            if (!relativeSuccess)
            {
                continue;
            }

            reconstructed.reference = refImage;
            reconstructed.next = nextImage;
            reconstructed.pose.setRotation(R);
            reconstructed.errorMax = errorMax;
        }
        else
        {
            // Try to fit an essential matrix (we assume we are approx. calibrated)
            Mat3 E;
            std::vector<size_t> inliers;
            const bool essentialSuccess = robustEssential(E,
                                                          inliers,
                                                          errorMax,
                                                          *refIntrinsics,
                                                          *nextIntrinsics,
                                                          refpts,
                                                          nextpts,
                                                          randomNumberGenerator,
                                                          countIterations,
                                                          minInliers,
                                                          distanceThreshold);
            if (!essentialSuccess)
            {
                continue;
            }

            std::vector<Vec3> structure;
            reconstructed.reference = refImage;
            reconstructed.next = nextImage;
            reconstructed.errorMax = errorMax;

            Mat4 T;
            if (!estimateTransformStructureFromEssential(T, structure, vecInliers, E, inliers, 
                                                      *refIntrinsics, *nextIntrinsics, 
                                                      refpts, nextpts))
            {
                continue;
            }

            if (vecInliers.size() < minInliers)
            {
                continue;
            }

            reconstructed.pose = geometry::Pose3(T);
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
