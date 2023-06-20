// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.


#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/config.hpp>
#include <boost/program_options.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfm/BundleAdjustmentCeres.hpp>
#include <aliceVision/sfm/BundleAdjustmentSymbolicCeres.hpp>

#include <string>
#include <sstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::sfm;

namespace po = boost::program_options;


void generateSampleSceneOnePlane(sfmData::SfMData & returnSfmDataGT, sfmData::SfMData & returnSfmDataToEstimate)
{
    sfmData::SfMData sfmDataGT;
    sfmData::SfMData sfmDataEst;

    auto phIntrinsicGT = camera::createIntrinsic(camera::PINHOLE_CAMERA_RADIAL3, 1920, 1080, 980, 980, 10, 20);
    auto phPinholeGT = std::dynamic_pointer_cast<camera::PinholeRadialK3>(phIntrinsicGT);
    std::vector<double> paramsGT = {0.1, 0.002, -0.01};
    phPinholeGT->setDistortionParams(paramsGT);
    sfmDataGT.getIntrinsics()[0] = phPinholeGT;

    auto phIntrinsicEst = camera::createIntrinsic(camera::PINHOLE_CAMERA_RADIAL3, 1920, 1080, 950, 950, 0, 0);
    auto phPinholeEst = std::dynamic_pointer_cast<camera::PinholeRadialK3>(phIntrinsicEst);
    std::vector<double> paramsEst = {0.0, 0.0, -0.0};
    phPinholeEst->setDistortionParams(paramsEst);
    sfmDataEst.getIntrinsics()[0] = phPinholeEst;

    Vec3 direction = {1.0, 1.0, 1.0};
    direction = direction.normalized();
    Vec3 axis = {1.0, 1.0, 0.0};
    axis = axis.normalized();

    for (int i = 0; i < 120; i++)
    {
        Vec3 pos = direction * double(i) / 120.0;
        Eigen::Matrix3d R = SO3::expm(axis * double(i) * M_PI / (8*120.0));
        geometry::Pose3 poseGT(R, pos);
        sfmData::CameraPose cposeGT(poseGT);
        sfmDataGT.getPoses()[i] = cposeGT;
        sfmDataGT.getViews()[i] = std::make_shared<sfmData::View>("", i, 0, i, 1920, 1080);

        Eigen::Matrix3d Rup = SO3::expm(Vec3::Random() * (0.1));
        Eigen::Vector3d tup = Vec3::Random() * (0.5);

        geometry::Pose3 poseEst(Rup * R, pos + tup);
        sfmData::CameraPose cposeEst(poseEst, (i==0));
        sfmDataEst.getPoses()[i] = cposeEst;
        sfmDataEst.getViews()[i] = std::make_shared<sfmData::View>("", i, 0, i, 1920, 1080);

    }

    int tid = 0;
    /*for (double y = -2.0; y < 2.0; y+=0.1)
    {
        for (double x = -2.0; x < 2.0; x+=0.1)
        {
            sfmData::Landmark lGT;
            lGT.X = Vec3(x, y, 2.0);
            lGT.descType = feature::EImageDescriberType::SIFT;
            sfmDataGT.getLandmarks()[tid] = lGT;

            sfmData::Landmark lEst = lGT;
            lEst.X += Vec3::Random() * 0.9;
            sfmDataEst.getLandmarks()[tid] = lEst;

            tid++;
        }
    }*/

    for (double y = -2.0; y < 2.0; y+=0.1)
    {
        for (double x = -2.0; x < 2.0; x+=0.1)
        {
            sfmData::Landmark lGT;
            lGT.X = Vec3(x, y, 4.0);
            lGT.descType = feature::EImageDescriberType::SIFT;
            sfmDataGT.getLandmarks()[tid] = lGT;

            sfmData::Landmark lEst = lGT;
            lEst.X += Vec3::Random() * 2.9;
            sfmDataEst.getLandmarks()[tid] = lEst;

            tid++;
        }
    }

    for (double y = -2.0; y < 2.0; y+=0.1)
    {
        for (double x = -2.0; x < 2.0; x+=0.1)
        {
            sfmData::Landmark lGT;
            lGT.X = Vec3(x, y, 3.0 + sqrt(x*x + y*y));
            lGT.descType = feature::EImageDescriberType::SIFT;
            sfmDataGT.getLandmarks()[tid] = lGT;

            sfmData::Landmark lEst = lGT;
            lEst.X += Vec3::Random() * 1.9;
            sfmDataEst.getLandmarks()[tid] = lEst;

            tid++;
        }
    }
    
    //Compute observations
    for (auto & pl : sfmDataGT.getLandmarks())
    {
        sfmData::Landmark & lEst = sfmDataEst.getLandmarks()[pl.first];
        
        for (auto & pp : sfmDataGT.getPoses())
        {
            sfmData::Observation obs;
            obs.x = phIntrinsicGT->project(pp.second.getTransform(), pl.second.X.homogeneous(), true);
            obs.scale = 1.0;
            obs.id_feat = pl.first;

            
            if (pp.second.getTransform()(pl.second.X)(2) < 0.1)
            {
                continue;
            }

            pl.second.observations[pp.first] = obs;
            lEst.observations[pp.first] = obs;
        }
    }

    returnSfmDataGT = sfmDataGT;
    returnSfmDataToEstimate = sfmDataEst;
}

int aliceVision_main(int argc, char **argv)
{
  po::options_description allParams("AliceVision sfmRegression");

  CmdLine cmdline("AliceVision sfmRegression");
  if (!cmdline.execute(argc, argv))
  {
      return EXIT_FAILURE;
  }

  {
    srand(0);
    sfmData::SfMData sfmDataGT;
    sfmData::SfMData sfmDataEst;
    generateSampleSceneOnePlane(sfmDataGT, sfmDataEst);

    BundleAdjustmentSymbolicCeres::CeresOptions options;
    BundleAdjustment::ERefineOptions refineOptions = BundleAdjustment::REFINE_ROTATION | BundleAdjustment::REFINE_TRANSLATION |BundleAdjustment::REFINE_STRUCTURE | BundleAdjustment::REFINE_INTRINSICS_FOCAL | BundleAdjustment::REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS | BundleAdjustment::REFINE_INTRINSICS_DISTORTION;
    options.summary = true;
    //options.nbThreads = 1;

    BundleAdjustmentSymbolicCeres BA(options, 3);
    const bool success = BA.adjust(sfmDataEst, refineOptions);
  }

  {
    srand(0);
    sfmData::SfMData sfmDataGT;
    sfmData::SfMData sfmDataEst;
    generateSampleSceneOnePlane(sfmDataGT, sfmDataEst);

    BundleAdjustmentCeres::CeresOptions options;
    BundleAdjustment::ERefineOptions refineOptions = BundleAdjustment::REFINE_ROTATION | BundleAdjustment::REFINE_TRANSLATION |BundleAdjustment::REFINE_STRUCTURE | BundleAdjustment::REFINE_INTRINSICS_FOCAL | BundleAdjustment::REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS | BundleAdjustment::REFINE_INTRINSICS_DISTORTION;
    options.summary = true;
    //options.nbThreads = 1;

    BundleAdjustmentCeres BA(options, 3);
    const bool success = BA.adjust(sfmDataEst, refineOptions);
  }

  return EXIT_SUCCESS;
}
