// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// This application tries to estimate the intrinsics and extrinsics of a set of images.
// It is assumed that for each image we have a result of the checkerboard detector.
// It is assumed that the distortion is at least approximately known or calibrated.
// It is assumed that we have several views with different poses orientation of the same checkerboard.
// It is assumed we know the square size of the checkerboard.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/calibration/checkerDetector.hpp>
#include <aliceVision/calibration/checkerDetector_io.hpp>
#include <aliceVision/calibration/distortionEstimation.hpp>

#include <aliceVision/robustEstimation/ACRansac.hpp>
#include <aliceVision/robustEstimation/IRansacKernel.hpp>
#include <aliceVision/multiview/RelativePoseKernel.hpp>
#include <aliceVision/multiview/ResectionKernel.hpp>
#include <aliceVision/multiview/Unnormalizer.hpp>
#include <aliceVision/multiview/relativePose/Homography4PSolver.hpp>
#include <aliceVision/multiview/resection/P3PSolver.hpp>
#include <aliceVision/multiview/resection/ProjectionDistanceError.hpp>
#include <aliceVision/multiview/relativePose/HomographyError.hpp>

#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustmentCeres.hpp>

#include <boost/program_options.hpp>

#include <fstream>
#include <map>
#include <utility>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

namespace po = boost::program_options;
using namespace aliceVision;

Eigen::Matrix<double, 1, 6> computeV(const Eigen::Matrix3d& H, int i, int j)
{
    Eigen::Matrix<double, 1, 6> v;

    v(0, 0) = H(0, i) * H(0, j);
    v(0, 1) = H(0, i) * H(1, j) + H(1, i) * H(0, j);
    v(0, 2) = H(1, i) * H(1, j);
    v(0, 3) = H(2, i) * H(0, j) + H(0, i) * H(2, j);
    v(0, 4) = H(2, i) * H(1, j) + H(1, i) * H(2, j);
    v(0, 5) = H(2, i) * H(2, j);

    return v;
}

bool estimateIntrinsicsPoses(sfmData::SfMData& sfmData, std::map<IndexT, calibration::CheckerDetector>& boardsAllImages, const double squareSize)
{
    if (boardsAllImages.size() < 2)
    {
        ALICEVISION_LOG_ERROR("At least 2 views are needed");
        return EXIT_FAILURE;
    }

    // Calibrate each intrinsic independently
    Eigen::Matrix<IndexT, Eigen::Dynamic, Eigen::Dynamic> indices;
    size_t global_checkerboard_w = 0;
    size_t global_checkerboard_h = 0;
    bool first = true;
    for (auto& pi : sfmData.getIntrinsics())
    {
        IndexT intrinsicId = pi.first;

        // Convert to pinhole
        std::shared_ptr<camera::IntrinsicBase>& intrinsicPtr = pi.second;
        std::shared_ptr<camera::Pinhole> cameraPinhole = std::dynamic_pointer_cast<camera::Pinhole>(intrinsicPtr);
        if (!cameraPinhole)
        {
            ALICEVISION_LOG_ERROR("Only work for pinhole cameras");
            return false;
        }

        ALICEVISION_LOG_INFO("Processing Intrinsic " << intrinsicId);

        std::map<std::pair<size_t, size_t>, int> checkerboard_dims;
        std::map<size_t, Eigen::Matrix3d> homographies;
        for (auto& pv : sfmData.getViews())
        {
            if (pv.second->getIntrinsicId() != intrinsicId)
                continue;

            const calibration::CheckerDetector& detector = boardsAllImages[pv.first];
            if (detector.getBoards().size() != 1)
            {
                ALICEVISION_LOG_WARNING("The view " << pv.first << " has either 0 or more than 1 checkerboard found.");
                continue;
            }

            // Build a list of points (meter to undistorted pixels)
            std::vector<Eigen::Vector2d> refpts;
            std::vector<Eigen::Vector2d> points;
            const auto& bs = detector.getBoards();
            const auto& b = bs[0];

            const auto dim = std::make_pair(size_t(b.cols()), size_t(b.rows()));
            if (checkerboard_dims.find(dim) == checkerboard_dims.end())
            {
                checkerboard_dims[dim] = 1;
            }
            else
            {
                ++checkerboard_dims[dim];
            }

            for (int i = 0; i < b.rows(); i++)
            {
                for (int j = 0; j < b.cols(); j++)
                {
                    IndexT cid = b(i, j);
                    if (b(i, j) == UndefinedIndexT)
                    {
                        continue;
                    }

                    Eigen::Vector2d refpt;
                    refpt(0) = double(j) * squareSize;
                    refpt(1) = double(i) * squareSize;

                    Eigen::Vector2d curpt;
                    curpt = detector.getCorners()[cid].center;
                    curpt = cameraPinhole->get_ud_pixel(curpt);

                    refpts.push_back(refpt);
                    points.push_back(curpt);
                }
            }

            // Estimate homography from this list of points
            Eigen::MatrixXd Mref(2, refpts.size());
            for (int idx = 0; idx < refpts.size(); idx++)
            {
                Mref(0, idx) = refpts[idx].x();
                Mref(1, idx) = refpts[idx].y();
            }

            Eigen::MatrixXd Mcur(2, points.size());
            for (int idx = 0; idx < points.size(); idx++)
            {
                Mcur(0, idx) = points[idx].x();
                Mcur(1, idx) = points[idx].y();
            }

            using KernelType = multiview::RelativePoseKernel<multiview::relativePose::Homography4PSolver,
                                                             multiview::relativePose::HomographyAsymmetricError,
                                                             multiview::UnnormalizerI,
                                                             robustEstimation::Mat3Model>;

            KernelType kernel(Mref, 1.0, 1.0, Mcur, cameraPinhole->w(), cameraPinhole->h(), false);  // configure as point to point error model.

            robustEstimation::Mat3Model model;
            std::mt19937 generator;
            std::vector<size_t> vec_inliers;

            robustEstimation::ACRANSAC(kernel, generator, vec_inliers, 1024, &model, std::numeric_limits<double>::infinity());
            Eigen::Matrix3d H = model.getMatrix();

            if (vec_inliers.size() < 10)
                continue;

            homographies[pv.first] = H;
        }

        // Now we use algorithm from [Zhang]
        Eigen::MatrixXd V(homographies.size() * 2, 6);

        size_t pos = 0;
        for (const auto& pH : homographies)
        {
            const Eigen::Matrix3d& H = pH.second;

            V.block<1, 6>(pos * 2, 0) = computeV(H, 0, 1);
            V.block<1, 6>(pos * 2 + 1, 0) = computeV(H, 0, 0) - computeV(H, 1, 1);
            pos++;
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(V, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::MatrixXd N = svd.matrixV();
        Eigen::VectorXd n = N.col(N.cols() - 1);

        Eigen::Matrix3d B;
        B(0, 0) = n(0);
        B(0, 1) = n(1);
        B(1, 0) = n(1);
        B(1, 1) = n(2);
        B(0, 2) = n(3);
        B(2, 0) = n(3);
        B(1, 2) = n(4);
        B(2, 1) = n(4);
        B(2, 2) = n(5);

        // Extract intrinsics from B
        double v0 = (B(0, 1) * B(0, 2) - B(0, 0) * B(1, 2)) / (B(0, 0) * B(1, 1) - B(0, 1) * B(0, 1));
        double lambda = B(2, 2) - (B(0, 2) * B(0, 2) + v0 * (B(0, 1) * B(0, 2) - B(0, 0) * B(1, 2))) / B(0, 0);
        double alpha = sqrt(lambda / B(0, 0));
        double beta = sqrt(lambda * B(0, 0) / (B(0, 0) * B(1, 1) - B(0, 1) * B(0, 1)));
        double gamma = -B(0, 1) * alpha * alpha * beta / lambda;
        double u0 = (gamma * v0 / beta) - (B(0, 2) * alpha * alpha / lambda);

        Eigen::Matrix3d A = Eigen::Matrix3d::Identity();
        A(0, 0) = alpha;
        A(1, 1) = beta;
        A(0, 1) = gamma;
        A(0, 2) = u0;
        A(1, 2) = v0;

        // Initialize camera intrinsics
        cameraPinhole->setK(A);

        // Estimate checkerboard dimensions
        size_t checkerboard_w = 0;
        size_t checkerboard_h = 0;
        int max_count = 0;
        for (const auto& [dim, count] : checkerboard_dims)
        {
            if (count > max_count)
            {
                max_count = count;
                checkerboard_w = dim.first;
                checkerboard_h = dim.second;
            }
        }

        if (first)
        {
            global_checkerboard_w = checkerboard_w;
            global_checkerboard_h = checkerboard_h;

            // Create landmarks
            indices = Eigen::Matrix<IndexT, Eigen::Dynamic, Eigen::Dynamic>(global_checkerboard_h, global_checkerboard_w);
            IndexT posLandmark = 0;
            for (int i = 0; i < global_checkerboard_h; i++)
            {
                for (int j = 0; j < global_checkerboard_w; j++)
                {
                    indices(i, j) = posLandmark;

                    sfmData::Landmark l(Vec3(squareSize * double(j), squareSize * double(i), 0.0), feature::EImageDescriberType::SIFT);
                    sfmData.getLandmarks()[posLandmark] = l;
                    posLandmark++;
                }
            }

            first = false;
        }
        else
        {
            if (checkerboard_w != global_checkerboard_w || checkerboard_h != global_checkerboard_h)
            {
                ALICEVISION_LOG_WARNING("Inconsistent checkerboard size.");
            }
        }

        // Initialize poses for each view using linear method
        Eigen::Matrix3d Ainv = A.inverse();
        for (const auto& pH : homographies)
        {
            const Eigen::Matrix3d& H = pH.second;
            Eigen::Matrix<double, 3, 4> T;
            Eigen::Matrix3d M;

            double tlambda = 1.0 / (Ainv * H.col(1)).norm();

            T.col(3) = tlambda * Ainv * H.col(2);
            if (T(2, 3) < 0.0)
            {
                tlambda *= -1.0;
            }
            T.col(3) = tlambda * Ainv * H.col(2);

            M.col(0) = tlambda * Ainv * H.col(0);
            M.col(1) = tlambda * Ainv * H.col(1);
            M.col(2) = M.col(0).cross(M.col(1));

            Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
            T.block<3, 3>(0, 0) = svd.matrixU() * svd.matrixV().transpose();

            geometry::Pose3 pose(T.block<3, 3>(0, 0), -T.block<3, 3>(0, 0).transpose() * T.col(3));
            sfmData::CameraPose cp;
            cp.setTransform(pose);

            auto view = sfmData.getViews().at(pH.first);
            sfmData.getPoses()[view->getPoseId()] = cp;
        }

        // Get residuals
        for (auto& pH : homographies)
        {
            IndexT viewId = pH.first;

            const calibration::CheckerDetector& detector = boardsAllImages[viewId];
            const auto& bs = detector.getBoards();
            const auto& b = bs[0];

            if (b.cols() != global_checkerboard_w || b.rows() != global_checkerboard_h)
            {
                continue;
            }

            const auto& corners = detector.getCorners();

            auto& curview = sfmData.getViews().at(viewId);

            for (int i = 0; i < global_checkerboard_h; i++)
            {
                for (int j = 0; j < global_checkerboard_w; j++)
                {
                    IndexT idxlandmark = indices(i, j);
                    if (idxlandmark == UndefinedIndexT)
                        continue;

                    IndexT idxcorner = b(i, j);
                    if (idxcorner == UndefinedIndexT)
                        continue;

                    Vec2 p = corners[idxcorner].center;

                    // Add observation
                    sfmData::Observation obs(p, idxlandmark, 1.0);
                    sfmData.getLandmarks()[idxlandmark].getObservations()[viewId] = obs;
                }
            }
        }

        // Compute non linear refinement
        sfm::BundleAdjustmentCeres::CeresOptions options;
        options.summary = true;
        sfm::BundleAdjustmentCeres ba(options);
        sfm::BundleAdjustment::ERefineOptions boptions = sfm::BundleAdjustment::ERefineOptions::REFINE_ROTATION |
                                                         sfm::BundleAdjustment::ERefineOptions::REFINE_TRANSLATION |
                                                         sfm::BundleAdjustment::ERefineOptions::REFINE_INTRINSICS_ALL;
        if (!ba.adjust(sfmData, boptions))
        {
            ALICEVISION_LOG_ERROR("Failed to calibrate");
            return false;
        }
    }

    return true;
}

bool estimateRigs(sfmData::SfMData& sfmData)
{
    // Calibrate rigs
    if (sfmData.getRigs().size() > 0)
    {
        // Initialize rig poses
        sfmData::Poses rigPoses;
        for (auto& pv : sfmData.getViews())
        {
            auto view = pv.second;
            if (view->isPartOfRig())
            {
                const IndexT rigId = view->getRigId();
                const IndexT subPoseId = view->getSubPoseId();
                sfmData::Rig& rig = sfmData.getRigs().at(rigId);
                sfmData::RigSubPose& subPose = rig.getSubPose(subPoseId);
                if (subPoseId == 0)
                {
                    if (sfmData.getPoses().find(view->getPoseId()) == sfmData.getPoses().end())
                    {
                        continue;
                    }
                    sfmData::CameraPose absPose = sfmData.getPoses().at(view->getPoseId());
                    rigPoses[rigId] = absPose;
                    subPose.pose = geometry::Pose3();  // identity
                    subPose.status = sfmData::ERigSubPoseStatus::CONSTANT;
                }
            }
        }

        // Initialize sub-poses
        for (auto& pv : sfmData.getViews())
        {
            auto view = pv.second;
            if (view->isPartOfRig())
            {
                const IndexT rigId = view->getRigId();
                const IndexT subPoseId = view->getSubPoseId();
                sfmData::Rig& rig = sfmData.getRigs().at(rigId);
                sfmData::RigSubPose& subPose = rig.getSubPose(subPoseId);
                if (subPoseId > 0)
                {
                    if (sfmData.getPoses().find(view->getPoseId()) == sfmData.getPoses().end())
                    {
                        continue;
                    }
                    sfmData::CameraPose absPose = sfmData.getPoses().at(view->getPoseId());
                    sfmData::CameraPose rigPose = rigPoses[rigId];
                    sfmData.getPoses()[view->getPoseId()] = rigPose;
                    subPose.pose = absPose.getTransform() * rigPose.getTransform().inverse();
                    subPose.status = sfmData::ERigSubPoseStatus::ESTIMATED;
                }
            }
        }

        // Turn off independent pose flag on views
        for (auto& pv : sfmData.getViews())
        {
            auto view = pv.second;
            if (view->isPartOfRig())
            {
                view->setIndependantPose(false);
            }
        }

        // Compute non-linear refinements
        sfm::BundleAdjustmentCeres::CeresOptions options;
        options.summary = true;
        sfm::BundleAdjustmentCeres ba(options);
        sfm::BundleAdjustment::ERefineOptions boptions = sfm::BundleAdjustment::ERefineOptions::REFINE_ROTATION |
                                                         sfm::BundleAdjustment::ERefineOptions::REFINE_TRANSLATION |
                                                         sfm::BundleAdjustment::ERefineOptions::REFINE_INTRINSICS_ALL;
        if (!ba.adjust(sfmData, boptions))
        {
            ALICEVISION_LOG_ERROR("Failed to calibrate");
            return false;
        }
    }

    return true;
}

int aliceVision_main(int argc, char* argv[])
{
    std::string sfmInputDataFilepath;
    std::string checkerBoardsPath;
    std::string sfmOutputDataFilepath;

    double squareSize = 10.;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmInputDataFilepath)->required(),
         "SfMData file input.")
        ("checkerboards,c", po::value<std::string>(&checkerBoardsPath)->required(),
         "Checkerboards json files directory.")
        ("output,o", po::value<std::string>(&sfmOutputDataFilepath)->required(),
         "SfMData file output.");
    
    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("squareSize,s", po::value<double>(&squareSize)->default_value(squareSize),
         "Checkerboard square width in mm.");
    // clang-format on

    CmdLine cmdline("This program calibrates camera intrinsics and extrinsics.\n"
                    "AliceVision checkerboardCalibration");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Load sfmData from disk
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmInputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    // Load the checkerboards
    std::map<IndexT, calibration::CheckerDetector> boardsAllImages;
    for (auto& pv : sfmData.getViews())
    {
        IndexT viewId = pv.first;

        // Read the json file
        std::stringstream ss;
        ss << checkerBoardsPath << "/"
           << "checkers_" << viewId << ".json";
        std::ifstream inputfile(ss.str());
        if (inputfile.is_open() == false)
            continue;

        std::stringstream buffer;
        buffer << inputfile.rdbuf();
        boost::json::value jv = boost::json::parse(buffer.str());

        // Store the checkerboard
        calibration::CheckerDetector detector(boost::json::value_to<calibration::CheckerDetector>(jv));
        boardsAllImages[viewId] = detector;
    }

    // Calibrate intrinsics and poses
    if (!estimateIntrinsicsPoses(sfmData, boardsAllImages, squareSize))
    {
        return EXIT_FAILURE;
    }

    // Calibrate rigs
    if (!estimateRigs(sfmData))
    {
        return EXIT_FAILURE;
    }

    // Mark all intrinsics as calibrated
    for (auto& pi : sfmData.getIntrinsics())
    {
        std::shared_ptr<camera::IntrinsicBase>& intrinsicPtr = pi.second;
        intrinsicPtr->setInitializationMode(camera::EInitMode::CALIBRATED);
    }

    // Mark all rig sub-poses as constant
    for (auto& pr : sfmData.getRigs())
    {
        sfmData::Rig& rig = pr.second;
        for (auto& subPose : rig.getSubPoses())
        {
            subPose.status = sfmData::ERigSubPoseStatus::CONSTANT;
        }
    }

    // Save sfmData to disk
    if (!sfmDataIO::save(sfmData, sfmOutputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmOutputDataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
