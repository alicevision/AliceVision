// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <boost/program_options.hpp>

#include <filesystem>
#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfmData;

namespace po = boost::program_options;
namespace fs = std::filesystem;

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string sfmDataFilename;
    std::string outDirectory;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&outDirectory)->required(),
         "Output folder.");
    // clang-format on

    CmdLine cmdline("AliceVision exportMVSTexturing");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    bool bOneHaveDisto = false;

    // Create output dir
    if (!fs::exists(outDirectory))
        fs::create_directory(outDirectory);

    // Read the SfM scene
    SfMData sfm_data;
    if (!sfmDataIO::load(sfm_data, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS | sfmDataIO::EXTRINSICS)))
    {
        std::cerr << std::endl << "The input SfMData file \"" << sfmDataFilename << "\" cannot be read." << std::endl;
        return EXIT_FAILURE;
    }

    for (Views::const_iterator iter = sfm_data.getViews().begin(); iter != sfm_data.getViews().end(); ++iter)
    {
        const View* view = iter->second.get();
        if (!sfm_data.isPoseAndIntrinsicDefined(view))
            continue;

        // Valid view, we can ask a pose & intrinsic data
        const Pose3 pose = sfm_data.getPose(*view).getTransform();
        Intrinsics::const_iterator iterIntrinsic = sfm_data.getIntrinsics().find(view->getIntrinsicId());
        const IntrinsicBase* cam = iterIntrinsic->second.get();

        if (!camera::isPinhole(cam->getType()))
            continue;

        const Pinhole* pinhole_cam = static_cast<const Pinhole*>(cam);

        // Extrinsic
        const Vec3& t = pose.translation();
        const Mat3& R = pose.rotation();
        // Intrinsic
        const double fx = pinhole_cam->getFocalLengthPixX();
        const double fy = pinhole_cam->getFocalLengthPixY();
        const Vec2 pp = pinhole_cam->getPrincipalPoint();

        // Image size in px
        const int w = pinhole_cam->w();
        const int h = pinhole_cam->h();

        // We can now create the .cam file for the View in the output dir
        std::ofstream outfile((fs::path(outDirectory) / (fs::path(view->getImage().getImagePath()).stem().string() + ".cam")).string());
        // See https://github.com/nmoehrle/mvs-texturing/blob/master/Arguments.cpp
        // for full specs
        const int largerDim = w > h ? w : h;
        outfile << t(0) << " " << t(1) << " " << t(2) << " " << R(0, 0) << " " << R(0, 1) << " " << R(0, 2) << " " << R(1, 0) << " " << R(1, 1) << " "
                << R(1, 2) << " " << R(2, 0) << " " << R(2, 1) << " " << R(2, 2) << "\n"
                << fx / largerDim << " 0 0 " << fx / fy << " " << pp(0) / w << " " << pp(1) / h;
        outfile.close();

        if (cam->hasDistortion())
            bOneHaveDisto = true;
    }

    const std::string sUndistMsg = bOneHaveDisto ? "undistorded" : "";
    const std::string sQuitMsg = std::string("Your SfMData file was succesfully converted!\n") + "Now you can copy your " + sUndistMsg +
                                 " images in the \"" + outDirectory + "\" folder and run MVS Texturing";
    std::cout << sQuitMsg << std::endl;
    return EXIT_SUCCESS;
}
