// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/fuseCut/InputSet.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/sfmData/SfMData.hpp>

#include <aliceVision/mvsUtils/MultiViewParams.hpp>

#include <aliceVision/fuseCut/BoundingBox.hpp>
#include <aliceVision/fuseCut/Fuser.hpp>
#include <aliceVision/fuseCut/PointCloud.hpp>
#include <aliceVision/fuseCut/GraphFiller.hpp>
#include <aliceVision/fuseCut/Mesher.hpp>

#include <aliceVision/mesh/Mesh.hpp>
#include <aliceVision/mesh/meshPostProcessing.hpp>

#include <boost/program_options.hpp>
#include <fstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

bool computeSubMesh(const std::string& pathSfmData, std::string& outputFile, const Eigen::Vector3d& bbMin, const Eigen::Vector3d& bbMax)
{
    // initialization
    StaticVector<StaticVector<int>> ptsCams;

    sfmData::SfMData sfmData;
    ALICEVISION_LOG_INFO("Loading source");
    if (!sfmDataIO::load(sfmData, pathSfmData, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << pathSfmData << "' cannot be read.");
        return false;
    }
    ALICEVISION_LOG_INFO("Loading source done");

    // Create multiview params
    mvsUtils::MultiViewParams mp(sfmData);
    mp.userParams.put("LargeScale.forcePixelSize", 0.01);
    mp.userParams.put("LargeScale.forceWeight", 32.0);
    mp.userParams.put("LargeScale.helperPointsGridSize", 10);
    mp.userParams.put("LargeScale.densifyNbFront", 0);
    mp.userParams.put("LargeScale.densifyNbBack", 0);
    mp.userParams.put("LargeScale.densifyScale", 1.0);
    mp.userParams.put("delaunaycut.seed", 0);
    mp.userParams.put("delaunaycut.nPixelSizeBehind", 4);
    mp.userParams.put("delaunaycut.fullWeight", 1.0);
    mp.userParams.put("delaunaycut.voteFilteringForWeaklySupportedSurfaces", true);
    mp.userParams.put("hallucinationsFiltering.invertTetrahedronBasedOnNeighborsNbIterations", 10);
    mp.userParams.put("hallucinationsFiltering.minSolidAngleRatio", 0.2);
    mp.userParams.put("hallucinationsFiltering.nbSolidAngleFilteringIterations", 2);

    // Set the input cams ids
    StaticVector<int> cams(mp.getNbCameras());
    for (int i = 0; i < cams.size(); ++i)
    {
        cams[i] = i;
    }

    if (cams.empty())
    {
        ALICEVISION_LOG_ERROR("No available camera/sensor");
        return false;
    }

    // Check bounding box validity
    double w = std::abs(bbMin.x() - bbMax.x());
    double h = std::abs(bbMin.y() - bbMax.y());
    double l = std::abs(bbMin.z() - bbMax.z());
    if (w < 1e-6 || h < 1e-6 || l < 1e-6)
        return true;

    // Use mp format
    std::array<Point3d, 8> lhexah;
    lhexah[0].x = bbMin.x();
    lhexah[0].y = bbMin.y();
    lhexah[0].z = bbMin.z();
    lhexah[1].x = bbMax.x();
    lhexah[1].y = bbMin.y();
    lhexah[1].z = bbMin.z();
    lhexah[2].x = bbMax.x();
    lhexah[2].y = bbMax.y();
    lhexah[2].z = bbMin.z();
    lhexah[3].x = bbMin.x();
    lhexah[3].y = bbMax.y();
    lhexah[3].z = bbMin.z();
    lhexah[4].x = bbMin.x();
    lhexah[4].y = bbMin.y();
    lhexah[4].z = bbMax.z();
    lhexah[5].x = bbMax.x();
    lhexah[5].y = bbMin.y();
    lhexah[5].z = bbMax.z();
    lhexah[6].x = bbMax.x();
    lhexah[6].y = bbMax.y();
    lhexah[6].z = bbMax.z();
    lhexah[7].x = bbMin.x();
    lhexah[7].y = bbMax.y();
    lhexah[7].z = bbMax.z();

    fuseCut::PointCloud pointcloud(mp);
    pointcloud.createDensePointCloud(&lhexah[0], cams, &sfmData, nullptr);

    // Cleanup sfmData
    sfmData.clear();

    ALICEVISION_LOG_INFO("Tetrahedralization");
    fuseCut::Tetrahedralization tetrahedralization(pointcloud.getVertices());
    ALICEVISION_LOG_INFO("Tetrahedralization done");

    fuseCut::GraphFiller gfiller(mp, pointcloud, tetrahedralization);
    // gfiller.setBoundingBox(bbMin, bbMax);

    ALICEVISION_LOG_INFO("build graph");
    gfiller.build(cams);

    ALICEVISION_LOG_INFO("binarize graph");
    gfiller.binarize();

    fuseCut::Mesher mesher(mp, pointcloud, tetrahedralization, gfiller.getCellsStatus());
    mesher.graphCutPostProcessing(&lhexah[0]);
    mesh::Mesh* mesh = mesher.createMesh(0);
    pointcloud.createPtsCams(ptsCams);
    mesh::meshPostProcessing(mesh, ptsCams, mp, "/", nullptr, &lhexah[0]);
    mesh->save(outputFile);
    delete mesh;

    return true;
}

int aliceVision_main(int argc, char* argv[])
{
    system::Timer timer;
    int rangeStart = -1;
    int rangeSize = 1;
    int rangeEnd = 1;

    std::string jsonFilename = "";
    std::string outputDirectory = "";
    std::string outputJsonFilename = "";

    fuseCut::BoundingBox boundingBox;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&jsonFilename)->required(),
         "Input JSON file.")
        ("output,o", po::value<std::string>(&outputDirectory)->required(),
         "Output directory for sub-meshes.")
        ("outputJson", po::value<std::string>(&outputJsonFilename)->required(),
         "Output scene description.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("boundingBox", po::value<fuseCut::BoundingBox>(&boundingBox),
         "Specifies a bounding box to reconstruct: position, rotation (Euler ZXY) and scale.")
        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
         "Range image index start.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
         "Range size.");
    // clang-format on

    CmdLine cmdline("AliceVision lidarMeshing");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Set maxThreads
    HardwareContext hwc = cmdline.getHardwareContext();

    std::ifstream inputfile(jsonFilename);
    if (!inputfile.is_open())
    {
        ALICEVISION_LOG_ERROR("Cannot open JSON input file");
        return EXIT_FAILURE;
    }

    std::stringstream buffer;
    buffer << inputfile.rdbuf();
    boost::json::value jv = boost::json::parse(buffer.str());
    fuseCut::InputSet inputsets(boost::json::value_to<fuseCut::InputSet>(jv));

    int setSize = static_cast<int>(inputsets.size());

    if (rangeStart != -1)
    {
        rangeSize = ceil(double(setSize) / double(rangeSize));
        rangeStart = rangeStart * rangeSize;
        rangeEnd = rangeStart + rangeSize;
        rangeEnd = std::min(rangeEnd, setSize);

        if (rangeStart > setSize)
        {
            ALICEVISION_LOG_INFO("Nothing to compute");
            return EXIT_SUCCESS;
        }
    }
    else
    {
        rangeStart = 0;
        rangeEnd = setSize;
    }

    for (int idSub = rangeStart; idSub < rangeEnd; idSub++)
    {
        const fuseCut::Input& input = inputsets[idSub];
        std::string ss = outputDirectory + "/subobj_" + std::to_string(idSub) + ".obj";

        ALICEVISION_LOG_INFO("Computing sub mesh " << idSub + 1 << " / " << setSize);
        if (!computeSubMesh(input.sfmPath, ss, input.bbMin, input.bbMax))
        {
            ALICEVISION_LOG_ERROR("Error computing sub mesh");
            return EXIT_FAILURE;
        }

        ALICEVISION_LOG_INFO(ss);
    }

    // Only the first chunk may update the json file
    if (rangeStart == 0)
    {
        for (int idSub = 0; idSub < setSize; idSub++)
        {
            const fuseCut::Input& input = inputsets[idSub];
            std::string ss = outputDirectory + "/subobj_" + std::to_string(idSub) + ".obj";
            inputsets[idSub].subMeshPath = ss;
        }

        std::ofstream of(outputJsonFilename);
        jv = boost::json::value_from(inputsets);
        of << boost::json::serialize(jv);
        of.close();
    }

    return EXIT_SUCCESS;
}
