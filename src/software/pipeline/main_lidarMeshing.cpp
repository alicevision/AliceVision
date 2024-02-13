// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
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
#include <aliceVision/fuseCut/Fuser.hpp>
#include <aliceVision/fuseCut/Octree.hpp>
#include <aliceVision/fuseCut/PointCloudBuilder.hpp>
#include <aliceVision/fuseCut/Tetrahedralization.hpp>
#include <aliceVision/fuseCut/GraphCut.hpp>
#include <aliceVision/fuseCut/GraphFiller.hpp>
#include <aliceVision/fuseCut/GCOutput.hpp>

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

bool computeSubMesh(const std::string & pathSfmData, std::string & outputFile)
{
    //initialization
    StaticVector<StaticVector<int>> ptsCams;

    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, pathSfmData, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << pathSfmData << "' cannot be read.");
        return false;
    }

    //Create multiview params
    mvsUtils::MultiViewParams mp(sfmData, "", "", "", false);
    mp.userParams.put("LargeScale.forcePixelSize", 0.01);
    mp.userParams.put("LargeScale.forceWeight", 32.0);

    //Set the input cams ids
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

    //Compute boundingbox
    Eigen::Vector3d bbMin, bbMax;
    sfmData.getBoundingBox(bbMin, bbMax);

    //Use mp format
    std::array<Point3d, 8> lhexah;
    lhexah[0].x = bbMin.x(); lhexah[0].y = bbMin.y(); lhexah[0].z = bbMin.z();
    lhexah[1].x = bbMax.x(); lhexah[1].y = bbMin.y(); lhexah[1].z = bbMin.z();
    lhexah[2].x = bbMax.x(); lhexah[2].y = bbMax.y(); lhexah[2].z = bbMin.z();
    lhexah[3].x = bbMin.x(); lhexah[3].y = bbMax.y(); lhexah[3].z = bbMin.z();
    lhexah[4].x = bbMin.x(); lhexah[4].y = bbMin.y(); lhexah[4].z = bbMax.z();
    lhexah[5].x = bbMax.x(); lhexah[5].y = bbMin.y(); lhexah[5].z = bbMax.z();
    lhexah[6].x = bbMax.x(); lhexah[6].y = bbMax.y(); lhexah[6].z = bbMax.z();
    lhexah[7].x = bbMin.x(); lhexah[7].y = bbMax.y(); lhexah[7].z = bbMax.z();

    //Build point cloud
    fuseCut::PointCloudBuilder builder(mp);
    builder.createDensePointCloud(&lhexah[0], cams, sfmData);

    //Tetrahedralize
    fuseCut::Tetrahedralization tetra;
    tetra.buildFromVertices(builder._verticesCoords);

    fuseCut::GraphFiller filler(mp);
    filler._tetrahedralization = tetra;
    filler._verticesCoords = builder._verticesCoords;
    filler._verticesAttr = builder._verticesAttr;
    filler.initCells();

    std::vector<fuseCut::Node::ptr> lnodes;
    builder._octree->visit(lnodes);
    std::cout << lnodes.size() << std::endl;
    if (lnodes.size() == 0) return false;

    std::set<std::pair<fuseCut::CellIndex, fuseCut::VertexIndex>> visited;
    filler.createGraphCut(lnodes[0]->getRayInfos(), *(lnodes[0]), visited);
    filler.forceTedgesByGradientIJCV(lnodes[0]->getRayInfos(), *(lnodes[0]));
    filler.final();

    fuseCut::GraphCut gc;
    gc._tetrahedralization = tetra;
    gc._verticesCoords = builder._verticesCoords;
    gc._verticesAttr = builder._verticesAttr;
    gc._cellsAttr = filler._cellsAttr;
    gc.maxflow();


    fuseCut::GCOutput output(mp);
    output._verticesAttr = filler._verticesAttr;
    output._tetrahedralization = filler._tetrahedralization;
    output._cellIsFull = gc._cellIsFull;
    output._verticesCoords = filler._verticesCoords;
    output._camsVertexes = builder._camsVertexes;

    output.graphCutPostProcessing(&lhexah[0], outputFile + "/");
    mesh::Mesh * mesh = output.createMesh(50);
    output.createPtsCams(ptsCams);
    mesh::meshPostProcessing(mesh, ptsCams, mp, outputFile + "/", nullptr, &lhexah[0]);

    mesh->save(outputFile);
    
    return true;
}

int aliceVision_main(int argc, char* argv[])
{
    system::Timer timer;
    int rangeStart = -1;
    int rangeSize = 1;

    std::string jsonFilename = "";
    std::string outputDirectory = "";
    std::string outputJsonFilename = "";

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&jsonFilename)->required(),
         "json file.")
        ("output,o", po::value<std::string>(&outputDirectory)->required(),
         "Output directory.")
        ("outputJson", po::value<std::string>(&outputJsonFilename)->required(),
         "Output scene description.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
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

    std::ifstream inputfile(jsonFilename);
    if (!inputfile.is_open())
    {
        ALICEVISION_LOG_ERROR("Cannot open json input file");
        return EXIT_FAILURE;
    }
    
    
    std::stringstream buffer;
    buffer << inputfile.rdbuf();
    boost::json::value jv = boost::json::parse(buffer.str());
    fuseCut::InputSet inputsets(boost::json::value_to<fuseCut::InputSet>(jv));

    size_t setSize = inputsets.size();
    if (rangeStart != -1)
    {
        if (rangeStart < 0 || rangeSize < 0)
        {
            ALICEVISION_LOG_ERROR("Range is incorrect");
            return EXIT_FAILURE;
        }

        if (rangeStart + rangeSize > setSize)
            rangeSize = setSize - rangeStart;

        if (rangeSize <= 0)
        {
            ALICEVISION_LOG_WARNING("Nothing to compute.");
            return EXIT_SUCCESS;
        }
    }
    else
    {
        rangeStart = 0;
        rangeSize = setSize;
    }

    for (int idSub = rangeStart; idSub < rangeStart + rangeSize; idSub++)
    {
        const fuseCut::Input & input = inputsets[idSub];
        std::string ss = outputDirectory + "/subobj_" + std::to_string(idSub) + ".obj";

        ALICEVISION_LOG_INFO("Computing sub mesh " << idSub + 1 << " / " << setSize);
        if (!computeSubMesh(input.sfmPath, ss))
        {
            ALICEVISION_LOG_ERROR("Error computing sub mesh");
            return EXIT_FAILURE;
        }

        inputsets[idSub].subMeshPath = ss;

        ALICEVISION_LOG_INFO(ss);
    }

    std::ofstream of(outputJsonFilename);
    jv = boost::json::value_from(inputsets);
    of << boost::json::serialize(jv);
    of.close();

    
    return EXIT_SUCCESS;
}
