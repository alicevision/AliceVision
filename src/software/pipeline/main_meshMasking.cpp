// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/jsonIO.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/mesh/Mesh.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/visibility.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>


// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;


template <class T>
auto optInRange(T min, T max, const char * opt_name)
{
    return [=] (T v)
    {
        if(v < min || v > max)
        {
            throw po::validation_error(po::validation_error::invalid_option_value, opt_name, std::to_string(v));
        }
    };
};

bool tryLoadMask(image::Image<unsigned char>* mask, const std::vector<std::string>& masksFolders, const IndexT viewId, const std::string& srcImage)
{
    for (const auto& masksFolder_str : masksFolders)
    {
        if (!masksFolder_str.empty() && fs::exists(masksFolder_str))
        {
            const auto masksFolder = fs::path(masksFolder_str);
            const auto idMaskPath = masksFolder / fs::path(std::to_string(viewId)).replace_extension("png");
            const auto nameMaskPath = masksFolder / fs::path(srcImage).filename().replace_extension("png");

            if (fs::exists(idMaskPath))
            {
                image::readImage(idMaskPath.string(), *mask, image::EImageColorSpace::LINEAR);
                return true;
            }
            else if (fs::exists(nameMaskPath))
            {
                image::readImage(nameMaskPath.string(), *mask, image::EImageColorSpace::LINEAR);
                return true;
            }
        }
    }
    return false;
}

void meshMasking(
    const mvsUtils::MultiViewParams & mp,
    const mesh::Mesh & inputMesh,
    const std::vector<std::string> & masksFolders,
    const std::string & outputMeshPath,
    int threshold,
    bool invert)
{
    // compute visibility for every vertex
    StaticVector<int> vertexVisibilityCounters;
    vertexVisibilityCounters.resize_with(inputMesh.pts.size(), 0);
    for (int camId = 0; camId < mp.getNbCameras(); ++camId)
    {
        image::Image<unsigned char> mask;
        if (!tryLoadMask(&mask, masksFolders, mp.getViewId(camId), mp.getImagePath(camId)))
        {
            continue;
        }

        if (mp.getWidth(camId) != mask.Width() || mp.getHeight(camId) != mask.Height())
        {
            ALICEVISION_LOG_WARNING("Invalid mask size: mask is ignored.");
            continue;
        }

        #pragma omp parallel for
        for (int vertexId = 0; vertexId < inputMesh.pts.size(); ++vertexId)
        {
            const auto& vertex = inputMesh.pts[vertexId];

            // check if the vertex is visible by the camera
            auto& pointVisibilities = inputMesh.pointsVisibilities[vertexId];
            const int pointVisibilityIndex = pointVisibilities.indexOf(camId);
            if (pointVisibilityIndex == -1)
            {
                continue;
            }

            // project vertex on mask
            Pixel projectedPixel;
            mp.getPixelFor3DPoint(&projectedPixel, vertex, camId);
            if (projectedPixel.x < 0 || projectedPixel.x >= mask.Width()
             || projectedPixel.y < 0 || projectedPixel.y >= mask.Height())
            {
                continue;
            }

            // get the mask value
            const bool maskValue = (mask(projectedPixel.y, projectedPixel.x) == 0);
            const bool masked = invert ? !maskValue : maskValue;
            if (!masked)
            {
                ++vertexVisibilityCounters[vertexId];
            }
        }
    }

    // filter masked vertex (remove adjacent triangles)
    StaticVector<int> visibleTriangles;
    visibleTriangles.reserve(inputMesh.tris.size() / 2);  // arbitrary size initial buffer
    for (int triangleId = 0; triangleId < inputMesh.tris.size(); ++triangleId)
    {
        const auto& triangle = inputMesh.tris[triangleId];
        const bool visible = std::all_of(std::begin(triangle.v), std::end(triangle.v), [&vertexVisibilityCounters, threshold](const int vertexId)
            {
                return vertexVisibilityCounters[vertexId] >= threshold;
            });
        if (visible)
        {
            visibleTriangles.push_back(triangleId);
        }
    }
    mesh::Mesh outMesh;
    mesh::PointVisibility outPointVisibility;
    inputMesh.generateMeshFromTrianglesSubset(visibleTriangles, outMesh, outPointVisibility);

    // Save output mesh
    outMesh.saveToObj(outputMeshPath);

    ALICEVISION_LOG_INFO("Mesh file: \"" << outputMeshPath << "\" saved.");
}


/**
 * @brief Write mask images from input images based on chosen algorithm.
 */
int main(int argc, char **argv)
{
    // command-line parameters
    std::string sfmFilePath;
    std::string inputMeshPath;
    std::vector<std::string> masksFolders;
    std::string outputMeshPath;
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());

    int threshold = 1;
    bool invert = false;

    po::options_description allParams("AliceVision masking");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmFilePath)->default_value(sfmFilePath)->required(),
            "A SfMData file (*.sfm).")
        ("inputMesh,i", po::value<std::string>(&inputMeshPath)->required(),
            "Input Mesh (OBJ file format).")
        ("masksFolders", po::value<std::vector<std::string>>(&masksFolders)->multitoken(),
            "Use masks from specific folder(s).\n"
            "Filename should be the same or the image uid.")
        ("outputMesh,o", po::value<std::string>(&outputMeshPath)->required(),
            "Output mesh (OBJ file format).")
        ("threshold", po::value<int>(&threshold)->default_value(threshold)->notifier(optInRange(1, INT_MAX, "threshold"))->required(),
            "The minimum number of visibility to keep a vertex.")
        ;

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("invert", po::value<bool>(&invert)->default_value(invert),
            "Invert the mask.")
        ;

    po::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
            "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(optionalParams).add(logParams);

    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, allParams), vm);

        if(vm.count("help") || (argc == 1))
        {
            ALICEVISION_COUT(allParams);
            return EXIT_SUCCESS;
        }
        po::notify(vm);
    }
    catch(boost::program_options::required_option& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    // check user choose at least one input option
    if(sfmFilePath.empty())
    {
        ALICEVISION_LOG_ERROR("Program need -i option" << std::endl << "No input images.");
        return EXIT_FAILURE;
    }

    // check input mesh
    ALICEVISION_LOG_INFO("Load input mesh.");
    mesh::Mesh inputMesh;
    if (!inputMesh.loadFromObjAscii(inputMeshPath))
    {
        ALICEVISION_LOG_ERROR("Unable to read input mesh from the file: " << inputMeshPath);
        return EXIT_FAILURE;
    }

    // check sfm file
    if(!sfmFilePath.empty() && !fs::exists(sfmFilePath) && !fs::is_regular_file(sfmFilePath))
    {
        ALICEVISION_LOG_ERROR("The input sfm file doesn't exist");
        return EXIT_FAILURE;
    }

    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmFilePath, sfmDataIO::ESfMData::ALL_DENSE))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" + sfmFilePath + "' cannot be read.");
        return EXIT_FAILURE;
    }

    // check output string
    if(outputMeshPath.empty())
    {
        ALICEVISION_LOG_ERROR("Invalid output");
        return EXIT_FAILURE;
    }

    // ensure output folder exists
    fs::path outputDirectory = fs::path(outputMeshPath).parent_path();
    if(!outputDirectory.empty() && !fs::exists(outputDirectory))
    {
        if(!fs::create_directory(outputDirectory))
        {
            ALICEVISION_LOG_ERROR("Cannot create output folder");
            return EXIT_FAILURE;
        }
    }

    // execute
    system::Timer timer;

    mvsUtils::MultiViewParams mp(sfmData);

    // load reference dense point cloud with visibilities
    ALICEVISION_LOG_INFO("Convert dense point cloud into ref mesh");
    mesh::Mesh refMesh;
    mvsUtils::createRefMeshFromDenseSfMData(refMesh, sfmData, mp);
    inputMesh.remapVisibilities(mesh::EVisibilityRemappingMethod::PullPush, refMesh);

    meshMasking(mp, inputMesh, masksFolders, outputMeshPath, threshold, invert);
    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
