// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/sfmStatistics.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustmentCeres.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/config.hpp>
#include <aliceVision/uncertainty/uncertainty.hpp>
#include <aliceVision/sfm/utils/gauge.hpp>
#include <aliceVision/system/Logger.hpp>

#include <boost/program_options.hpp>

#include <algorithm>
#include <string>
#include <sstream>
#include <vector>
#include <array>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::sfm;
using namespace aliceVision::sfmData;
using namespace aliceVision::sfmDataIO;
namespace po = boost::program_options;


int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmDataFilename;
    std::string outSfMDataFilename;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file to align.")
        ("output,o", po::value<std::string>(&outSfMDataFilename)->required(),
         "Output SfMData scene.");
    // clang-format on

    CmdLine cmdline("AliceVision computeUncertainty");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }


    // Load input scene
    SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmDataFilename, ESfMData(ALL)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file " << sfmDataFilename << " cannot be read.");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Finding points to lock");
    std::array<IndexT, 3> sample;
    if (!selectTripletForGaugeRemoval(sfmData, sample))
    {
        ALICEVISION_LOG_ERROR("Failed to find initial set of point");
        return EXIT_FAILURE;
    }    

    std::array<bool, 3> previouslyLocked;
    for (int i = 0; i < 3; i++)
    {
        previouslyLocked[i] = sfmData.getLandmarks().at(sample[i]).isLocked();
        sfmData.getLandmarks().at(sample[i]).setLocked(true);
    }

    // Using the bundle adjustment problem formulation
    // Compute the jacobian as if we were doing another computation step
    // Except we don't use the loss function.
    ALICEVISION_LOG_INFO("Computing jacobian");
    std::map<IndexT, size_t> poseToPosition;
    std::map<IndexT, size_t> intrinsicsToPosition;
    std::map<IndexT, size_t> distortionToPosition;
    std::map<IndexT, size_t> landmarkToPosition;
    ceres::CRSMatrix jacobian;
    BundleAdjustmentCeres bundleAdjustmentObj;
    bundleAdjustmentObj.createJacobian(sfmData, jacobian, poseToPosition, intrinsicsToPosition, distortionToPosition, landmarkToPosition);


    // Make an Eigen view on the jacobian
    ALICEVISION_LOG_INFO("Building map to Ceres");
    Eigen::Map<const Eigen::SparseMatrix<double, Eigen::RowMajor, int>> J_eigen(
        jacobian.num_rows,
        jacobian.num_cols,
        static_cast<int>(jacobian.values.size()),
        jacobian.rows.data(),    // outer index ptr (row offsets), size = num_rows + 1
        jacobian.cols.data(),    // inner indices (col ids), size = nnz
        jacobian.values.data()   // values, size = nnz
    );

    Eigen::MatrixXd covarianceCameras;
    bool res = uncertainty::computeUncertainty(covarianceCameras, J_eigen, landmarkToPosition.size());
    if (!res)
    {
        return EXIT_FAILURE;
    }

    for (int i = 0; i < 3; i++)
    {        
        sfmData.getLandmarks().at(sample[i]).setLocked(previouslyLocked[i]);
    }

    // Store covariance
    for (const auto & [index, position] : poseToPosition)
    {
        sfmData.setPoseUncertainty(index, covarianceCameras.block<6, 6>(position * 6, position * 6));
    }

    ALICEVISION_LOG_INFO("Export SfM: " << outSfMDataFilename);
    if (!sfmDataIO::save(sfmData, outSfMDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << outSfMDataFilename << "' cannot be written.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
