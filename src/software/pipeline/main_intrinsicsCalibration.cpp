// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/cmdline.hpp>
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
#include <aliceVision/multiview/Unnormalizer.hpp>
#include <aliceVision/multiview/relativePose/Homography4PSolver.hpp>
#include <aliceVision/multiview/relativePose/HomographyError.hpp>

#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/sfm/BundleAdjustmentSymbolicCeres.hpp>


#include <boost/program_options.hpp>

#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

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


int aliceVision_main(int argc, char* argv[]) 
{
    std::string sfmInputDataFilepath;
    std::string checkerBoardsPath;
    std::string sfmOutputDataFilepath;
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    const double squareSize = 0.02;

    // Command line parameters
    po::options_description allParams(
        "Parse external information about cameras used in a panorama.\n"
        "AliceVision PanoramaInit");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmInputDataFilepath)->required(), "SfMData file input.")
        ("checkerboards", po::value<std::string>(&checkerBoardsPath)->required(), "Checkerboards json files directory.")
        ("outSfMData,o", po::value<std::string>(&sfmOutputDataFilepath)->required(), "SfMData file output.")
        ;

    po::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
            "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(logParams);

    // Parse command line
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

    system::Logger::get()->setLogLevel(verboseLevel);

    //Load sfmData from disk
    sfmData::SfMData sfmData;
    if (!sfmDataIO::Load(sfmData, sfmInputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }


    //Load the checkerboards
    std::map < IndexT, calibration::CheckerDetector> boardsAllImages;
    for (auto& pv : sfmData.getViews())
    {
        IndexT viewId = pv.first;

        // Read the json file
        std::stringstream ss;
        ss << checkerBoardsPath << "/" << "checkers_" << viewId << ".json";
        std::ifstream inputfile(ss.str());
        if (inputfile.is_open() == false)
        {
            continue;
        }

        std::stringstream buffer;
        buffer << inputfile.rdbuf();
        boost::json::value jv = boost::json::parse(buffer.str());

        //Store the checkerboard
        calibration::CheckerDetector detector(boost::json::value_to<calibration::CheckerDetector>(jv));
        boardsAllImages[viewId] = detector;
    }

    //Calibrate each intrinsic independently
    Eigen::Matrix<IndexT, Eigen::Dynamic, Eigen::Dynamic> indices;
    size_t global_max_checkerboard_w = 0;
    size_t global_max_checkerboard_h = 0;
    bool first = true;
    for (auto& pi : sfmData.getIntrinsics())
    {
        IndexT intrinsicId = pi.first;

        //Convert to pinhole
        std::shared_ptr<camera::IntrinsicBase>& intrinsicPtr = pi.second;
        std::shared_ptr<camera::Pinhole> cameraPinhole = std::dynamic_pointer_cast<camera::Pinhole>(intrinsicPtr);
        if (!cameraPinhole)
        {
            ALICEVISION_LOG_ERROR("Only work for pinhole cameras");
            return EXIT_FAILURE;
        }
        ALICEVISION_LOG_INFO("Processing Intrinsic " << intrinsicId);


        size_t max_checkerboard_w = 0;
        size_t max_checkerboard_h = 0;
        std::map<size_t, Eigen::Matrix3d> homographies;
        for (auto& pv : sfmData.getViews())
        {
            if (pv.second->getIntrinsicId() != intrinsicId)
            {
                continue;
            }

            const calibration::CheckerDetector & detector = boardsAllImages[pv.first];
            if (detector.getBoards().size() != 1)
            {
                ALICEVISION_LOG_ERROR("The view " << pv.first << " has either 0 or more than 1 checkerboard found.");
                continue;
            }

            //Build a list of points (meter to undistorted pixels)
            std::vector<Eigen::Vector2d> refpts;
            std::vector<Eigen::Vector2d> points;
            const auto & bs = detector.getBoards();
            const auto& b = bs[0];

            max_checkerboard_w = std::max(size_t(b.cols()), max_checkerboard_w);
            max_checkerboard_h = std::max(size_t(b.rows()), max_checkerboard_h);

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

            //Estimate homography from this list of points
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
                multiview::UnnormalizerI, robustEstimation::Mat3Model>;

            KernelType kernel(Mref, 1.0, 1.0, Mcur, cameraPinhole->w(), cameraPinhole->h(), false); // configure as point to point error model.

            robustEstimation::Mat3Model model;
            std::mt19937 generator;
            std::vector<size_t> vec_inliers;

            robustEstimation::ACRANSAC(kernel, generator, vec_inliers, 1024, &model, std::numeric_limits<double>::infinity());
            Eigen::Matrix3d H = model.getMatrix();

            if (vec_inliers.size() < 10)
            {
                continue;
            }

            homographies[pv.first] = H;
        }
        
       
        
        //Now we use algorithm from Zhang, A flexible new technique for camera calibration
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


        //Extract intrinsics from B
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

        //Initialize camera intrinsics
        cameraPinhole->setK(A);

        if (first)
        {
            //Create landmarks
            indices = Eigen::Matrix<IndexT, Eigen::Dynamic, Eigen::Dynamic>(max_checkerboard_h, max_checkerboard_w);
            IndexT posLandmark = 0;
            for (int i = 0; i < max_checkerboard_h; i++)
            {
                for (int j = 0; j < max_checkerboard_w; j++)
                {
                    indices(i, j) = posLandmark;

                    sfmData::Landmark l(Vec3(squareSize * double(j), squareSize * double(i), 0.0), feature::EImageDescriberType::SIFT);
                    sfmData.getLandmarks()[posLandmark] = l;
                    posLandmark++;
                }
            }

            global_max_checkerboard_w = max_checkerboard_w;
            global_max_checkerboard_h = max_checkerboard_h;
            first = false;
        }
        else
        {
            if (global_max_checkerboard_h != max_checkerboard_h || global_max_checkerboard_w != max_checkerboard_w)
            {
                ALICEVISION_LOG_ERROR("Inconsistent checkerboard size");
                return EXIT_FAILURE;
            }
        }
        

        //Initialize poses for each view using linear method
        Eigen::Matrix3d Ainv = A.inverse();
        for (const auto& pH : homographies)
        {
            const Eigen::Matrix3d& H = pH.second;
            Eigen::Matrix<double, 3, 4> T;
            Eigen::Matrix3d M;

            Eigen::Vector3d L = Ainv * H.col(1);
            double tlambda = 1.0 / L.norm();

            M.col(0) = tlambda * Ainv * H.col(0);
            M.col(1) = tlambda * Ainv * H.col(1);
            M.col(2) = M.col(0).cross(M.col(1));

            Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
            T.block<3, 3>(0, 0) = svd.matrixU() * svd.matrixV().transpose();

            T.col(3) = tlambda * Ainv * H.col(2);

            geometry::Pose3 pose(T);
            sfmData::CameraPose cp;
            cp.setTransform(pose);
            sfmData.getPoses()[pH.first] = cp;
        }


        //Get residuals
        for (auto& pH : homographies)
        {
            IndexT viewId = pH.first;

            const calibration::CheckerDetector& detector = boardsAllImages[viewId];
            const auto& bs = detector.getBoards();
            const auto& b = bs[0];
            const auto& corners = detector.getCorners();

            auto & curview = sfmData.getViews()[viewId];

            for (int i = 0; i < b.rows(); i++)
            {
                for (int j = 0; j < b.cols(); j++)
                {
                    IndexT idxlandmark = indices(i, j);
                    if (idxlandmark == UndefinedIndexT)
                        continue;

                    IndexT idxcorner = b(i, j);
                    if (idxcorner == UndefinedIndexT)
                        continue;

                    Vec2 p = corners[idxcorner].center;

                    const Vec2 cpt = cameraPinhole->ima2cam(p);
                    const Vec2 distorted = cameraPinhole->removeDistortion(cpt);
                    const Vec2 ipt = cameraPinhole->cam2ima(distorted);

                    //Add observation
                    sfmData::Observation obs(p, idxlandmark, 1.0);
                    sfmData.getLandmarks()[idxlandmark].observations[viewId] = obs;
                }
            }
        }

        //Compute non linear refinement
        sfm::BundleAdjustmentSymbolicCeres::CeresOptions options;
        /*options.summary = true;
        options.nbThreads = 1;*/
        sfm::BundleAdjustmentSymbolicCeres ba(options);
        sfm::BundleAdjustment::ERefineOptions boptions = sfm::BundleAdjustment::ERefineOptions::REFINE_ROTATION | 
                                                        sfm::BundleAdjustment::ERefineOptions::REFINE_TRANSLATION | 
                                                        sfm::BundleAdjustment::ERefineOptions::REFINE_INTRINSICS_ALL;
        if (!ba.adjust(sfmData, boptions))
        {
            ALICEVISION_LOG_ERROR("Failed to calibrate");
            return EXIT_FAILURE;
        }
    }

    //Save sfmData to disk
    if (!sfmDataIO::Save(sfmData, sfmOutputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmOutputDataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }

	return EXIT_SUCCESS;
}