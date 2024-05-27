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

#include <aliceVision/camera/Pinhole.hpp>

#include <aliceVision/robustEstimation/ACRansac.hpp>
#include <aliceVision/multiview/essential.hpp>
#include <aliceVision/multiview/relativePose/FundamentalError.hpp>
#include <aliceVision/multiview/RelativePoseKernel.hpp>
#include <aliceVision/multiview/triangulation/triangulationDLT.hpp>
#include <aliceVision/multiview/relativePose/Rotation3PSolver.hpp>
#include <aliceVision/multiview/relativePose/Essential5PSolver.hpp>

#include <aliceVision/geometry/lie.hpp>

#include <boost/program_options.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>

#include <aliceVision/sfm/pipeline/relativePoses.hpp>

#include <cstdlib>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

bool getPoseStructure(Mat3& R,
                      Vec3& t,
                      std::vector<Vec3>& structure,
                      std::vector<size_t>& newVecInliers,
                      const Mat3& E,
                      const std::vector<size_t>& vecInliers,
                      const Mat3& K1,
                      const Mat3& K2,
                      const Mat& x1,
                      const Mat& x2)
{
    // Find set of analytical solutions
    std::vector<Mat3> Rs;
    std::vector<Vec3> ts;
    motionFromEssential(E, &Rs, &ts);

    Mat34 P1, P2;
    Mat3 R1 = Mat3::Identity();
    Vec3 t1 = Vec3::Zero();
    P_from_KRt(K1, R1, t1, P1);

    size_t bestCoundValid = 0;

    for (int it = 0; it < Rs.size(); it++)
    {
        const Mat3& R2 = Rs[it];
        const Vec3& t2 = ts[it];

        P_from_KRt(K2, R2, t2, P2);

        std::vector<Vec3> points;
        std::vector<size_t> updatedInliers;

        size_t countValid = 0;
        for (size_t k = 0; k < vecInliers.size(); ++k)
        {
            const Vec2& pt1 = x1.col(vecInliers[k]);
            const Vec2& pt2 = x2.col(vecInliers[k]);

            Vec3 X;
            multiview::TriangulateDLT(P1, pt1, P2, pt2, X);

            // Test if point is front to the two cameras.
            if (Depth(R1, t1, X) > 0 && Depth(R2, t2, X) > 0)
            {
                countValid++;
            }

            updatedInliers.push_back(vecInliers[k]);
            points.push_back(X);
        }

        if (countValid > bestCoundValid)
        {
            bestCoundValid = countValid;
            structure = points;
            newVecInliers = updatedInliers;
            R = Rs[it];
            t = ts[it];
        }
    }

    if (newVecInliers.size() < 10)
    {
        return false;
    }

    return true;
}

bool robustEssential(Mat3& E,
                     std::vector<size_t>& vecInliers,
                     const camera::IntrinsicBase & cam1,
                     const camera::IntrinsicBase & cam2,
                     const Mat& x1,
                     const Mat& x2,
                     std::mt19937& randomNumberGenerator,
                     const size_t maxIterationCount,
                     const size_t minInliers)
{
    // use the 5 point solver to estimate E
    using SolverT = multiview::relativePose::Essential5PSolver;

    // define the kernel
    using KernelT =
      multiview::RelativePoseKernel_K<SolverT, multiview::relativePose::FundamentalSymmetricEpipolarDistanceError, robustEstimation::Mat3Model>;

    const camera::Pinhole & pinhole1 = dynamic_cast<const camera::Pinhole &>(cam1);
    const camera::Pinhole & pinhole2 = dynamic_cast<const camera::Pinhole &>(cam2);

    KernelT kernel(x1, cam1.w(), cam1.h(), x2, cam2.w(), cam2.h(), pinhole1.K(), pinhole2.K());

    robustEstimation::Mat3Model model;
    vecInliers.clear();

    // robustly estimation of the Essential matrix and its precision
    const std::pair<double, double> acRansacOut =
      robustEstimation::ACRANSAC(kernel, randomNumberGenerator, vecInliers, maxIterationCount, &model);

    if (vecInliers.size() < minInliers)
    {
        return false;
    }

    E = model.getMatrix();

    return true;
}

bool robustRotation(Mat3& R,
                    std::vector<size_t>& vecInliers,
                    const Mat& x1,
                    const Mat& x2,
                    std::mt19937& randomNumberGenerator,
                    const size_t maxIterationCount,
                    const size_t minInliers)
{
    using KernelType = multiview::
      RelativePoseSphericalKernel<multiview::relativePose::Rotation3PSolver, multiview::relativePose::RotationError, robustEstimation::Mat3Model>;

    KernelType kernel(x1, x2);

    robustEstimation::Mat3Model model;
    vecInliers.clear();

    // robustly estimation of the Essential matrix and its precision
    robustEstimation::ACRANSAC(kernel, randomNumberGenerator, vecInliers, 1024, &model, std::numeric_limits<double>::infinity());

    if (vecInliers.size() < minInliers)
    {
        return false;
    }

    R = model.getMatrix();

    return true;
}

void computeCovisibility(std::map<Pair, unsigned int>& covisibility, const track::TracksMap& mapTracks)
{
    for (const auto& item : mapTracks)
    {
        const auto& track = item.second;

        for (auto it = track.featPerView.begin(); it != track.featPerView.end(); it++)
        {
            Pair p;
            p.first = it->first;

            for (auto next = std::next(it); next != track.featPerView.end(); next++)
            {
                p.second = next->first;

                if (covisibility.find(p) == covisibility.end())
                {
                    covisibility[p] = 0;
                }
                else
                {
                    covisibility[p]++;
                }
            }
        }
    }
}

double computeAreaScore(const std::vector<Eigen::Vector2d>& refPts, const std::vector<Eigen::Vector2d>& nextPts, double refArea, double nextArea)
{
    namespace bg = boost::geometry;

    typedef boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian> point_t;
    typedef boost::geometry::model::multi_point<point_t> mpoint_t;
    typedef boost::geometry::model::polygon<point_t> polygon;
    mpoint_t mpt1, mpt2;

    for (int idx = 0; idx < refPts.size(); idx++)
    {
        const auto& refPt = refPts[idx];
        const auto& nextPt = nextPts[idx];

        boost::geometry::append(mpt1, point_t(refPt(0), refPt(1)));
        boost::geometry::append(mpt2, point_t(nextPt(0), nextPt(1)));
    }

    polygon hull1, hull2;
    boost::geometry::convex_hull(mpt1, hull1);
    boost::geometry::convex_hull(mpt2, hull2);
    double area1 = boost::geometry::area(hull1);
    double area2 = boost::geometry::area(hull1);
    double score = (area1 + area2) / (refArea + nextArea);

    return score;
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
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("tracksFilename,t", po::value<std::string>(&tracksFilename)->required(),
         "Tracks file.")
        ("output,o", po::value<std::string>(&outputDirectory)->required(),
         "Path to the output directory.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("enforcePureRotation,e", po::value<bool>(&enforcePureRotation)->default_value(enforcePureRotation),
         "Enforce pure rotation in estimation.")
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
         "Range image index start.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
         "Range size.");
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
    std::ifstream tracksFile(tracksFilename);
    if (tracksFile.is_open() == false)
    {
        ALICEVISION_LOG_ERROR("The input tracks file '" + tracksFilename + "' cannot be read.");
        return EXIT_FAILURE;
    }
    std::stringstream buffer;
    buffer << tracksFile.rdbuf();
    boost::json::value jv = boost::json::parse(buffer.str());
    track::TracksMap mapTracks(track::flat_map_value_to<track::Track>(jv));

    // Compute tracks per view
    ALICEVISION_LOG_INFO("Estimate tracks per view");
    track::TracksPerView mapTracksPerView;
    for (const auto& viewIt : sfmData.getViews())
    {
        // create an entry in the map
        mapTracksPerView[viewIt.first];
    }
    track::computeTracksPerView(mapTracks, mapTracksPerView);

    ALICEVISION_LOG_INFO("Compute co-visibility");
    std::map<Pair, unsigned int> covisibility;
    computeCovisibility(covisibility, mapTracks);

    ALICEVISION_LOG_INFO("Process co-visibility");
    std::stringstream ss;
    ss << outputDirectory << "/pairs_" << rangeStart << ".json";
    std::ofstream of(ss.str());

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
        std::shared_ptr<camera::Pinhole> refPinhole = std::dynamic_pointer_cast<camera::Pinhole>(refIntrinsics);
        std::shared_ptr<camera::Pinhole> nextPinhole = std::dynamic_pointer_cast<camera::Pinhole>(nextIntrinsics);

        aliceVision::track::TracksMap mapTracksCommon;
        track::getCommonTracksInImagesFast({refImage, nextImage}, mapTracks, mapTracksPerView, mapTracksCommon);

        // Build features coordinates matrices
        const std::size_t n = mapTracksCommon.size();
        Mat refX(2, n);
        Mat nextX(2, n);
        IndexT pos = 0;
        for (const auto& commonItem : mapTracksCommon)
        {
            const track::Track& track = commonItem.second;

            refX.col(pos) = track.featPerView.at(refImage).coords;
            nextX.col(pos) = track.featPerView.at(nextImage).coords;

            pos++;
        }

        std::vector<size_t> vecInliers;
        sfm::ReconstructedPair reconstructed;

        if (enforcePureRotation)
        {
            // Transform to vector
            Mat refVecs(3, n);
            Mat nextVecs(3, n);
            for (int idx = 0; idx < n; idx++)
            {
                // Lift to unit sphere
                refVecs.col(idx) = refIntrinsics->toUnitSphere(refIntrinsics->removeDistortion(refIntrinsics->ima2cam(refX.col(idx))));
                nextVecs.col(idx) = nextIntrinsics->toUnitSphere(nextIntrinsics->removeDistortion(nextIntrinsics->ima2cam(nextX.col(idx))));
            }

            // Try to fit an essential matrix (we assume we are approx. calibrated)
            Mat3 R;
            const bool relativeSuccess = robustRotation(R, vecInliers, refVecs, nextVecs, randomNumberGenerator, 1024, minInliers);
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
                                                          refX,
                                                          nextX,
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

            if (!getPoseStructure(
                  reconstructed.R, reconstructed.t, structure, vecInliers, E, inliers, refPinhole->K(), nextPinhole->K(), refX, nextX))
            {
                continue;
            }
        }

        std::vector<Vec2> refpts, nextpts;
        for (auto id : vecInliers)
        {
            refpts.push_back(refX.col(id));
            nextpts.push_back(nextX.col(id));
        }

        // Compute matched points coverage of image
        double areaRef = refIntrinsics->w() * refIntrinsics->h();
        double areaNext = nextIntrinsics->w() * nextIntrinsics->h();
        double areaScore = computeAreaScore(refpts, nextpts, areaRef, areaNext);

        // Compute ratio of matched points
        double iunion = n;
        double iinter = vecInliers.size();
        double score = iinter / iunion;
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
