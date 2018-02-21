// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/structures/Point3d.hpp>
#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/common/common.hpp>
#include <aliceVision/common/MultiViewParams.hpp>
#include <aliceVision/common/PreMatchCams.hpp>
#include <aliceVision/mesh/meshPostProcessing.hpp>
#include <aliceVision/meshConstruction/LargeScale.hpp>
#include <aliceVision/meshConstruction/ReconstructionPlan.hpp>
#include <aliceVision/meshConstruction/DelaunayGraphCut.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

using namespace aliceVision;
namespace bfs = boost::filesystem;
namespace po = boost::program_options;

#define ALICEVISION_COUT(x) std::cout << x << std::endl
#define ALICEVISION_CERR(x) std::cerr << x << std::endl

enum EPartitioning {
    eUndefined = 0,
    eSingleBlock = 1,
    eAuto = 2
};

EPartitioning EPartitioning_stringToEnum(const std::string& s)
{
    if(s == "singleBlock")
        return eSingleBlock;
    if(s == "auto")
        return eAuto;
    return eUndefined;
}

inline std::istream& operator>>(std::istream& in, EPartitioning& mode)
{
    std::string s;
    in >> s;
    mode = EPartitioning_stringToEnum(s);
    return in;
}


int main(int argc, char* argv[])
{
    long startTime = clock();

    std::string iniFilepath;
    std::string outputMesh;
    std::string depthMapFolder;
    std::string depthMapFilterFolder;
    EPartitioning partitioning = eSingleBlock;
    po::options_description inputParams;
    int maxPts = 6000000;
    int maxPtsPerVoxel = 6000000;

    po::options_description allParams("AliceVision meshing");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("ini", po::value<std::string>(&iniFilepath)->required(),
            "Configuration file (mvs.ini).")
        ("depthMapFolder", po::value<std::string>(&depthMapFolder)->required(),
            "Input depth maps folder.")
        ("depthMapFilterFolder", po::value<std::string>(&depthMapFilterFolder)->required(),
            "Input filtered depth maps folder.")
        ("output,o", po::value<std::string>(&outputMesh)->required(),
            "Output mesh (OBJ file format).");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("maxPts", po::value<int>(&maxPts)->default_value(maxPts),
            "Max points.")
        ("maxPtsPerVoxel", po::value<int>(&maxPtsPerVoxel)->default_value(maxPtsPerVoxel),
            "Max points per voxel.")
        ("partitioning", po::value<EPartitioning>(&partitioning)->default_value(partitioning),
            "Partitioning: singleBlock or auto.");

    allParams.add(requiredParams).add(optionalParams);

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
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << allParams);
      return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << allParams);
      return EXIT_FAILURE;
    }

    ALICEVISION_COUT("ini file: " << iniFilepath);

    // .ini parsing
    common::MultiViewInputParams mip(iniFilepath, depthMapFolder, depthMapFilterFolder);
    const double simThr = mip._ini.get<double>("global.simThr", 0.0);
    common::MultiViewParams mp(mip.getNbCameras(), &mip, (float) simThr);
    common::PreMatchCams pc(&mp);

    // .ini parsing
    int ocTreeDim = mip._ini.get<int>("LargeScale.gridLevel0", 1024);
    const auto baseDir = mip._ini.get<std::string>("LargeScale.baseDirName", "root01024");

    bfs::path outDirectory = bfs::path(outputMesh).parent_path();
    if(!bfs::is_directory(outDirectory))
        bfs::create_directory(outDirectory);

    bfs::path tmpDirectory = outDirectory / "tmp";

    switch(partitioning)
    {
        case eAuto:
        {
            ALICEVISION_COUT("--- meshing partitioning: auto");
            meshConstruction::LargeScale lsbase(&mp, &pc, tmpDirectory.string() + "/");
            lsbase.generateSpace(maxPtsPerVoxel, ocTreeDim);
            std::string voxelsArrayFileName = lsbase.spaceFolderName + "hexahsToReconstruct.bin";
            StaticVector<Point3d>* voxelsArray = nullptr;
            if(bfs::exists(voxelsArrayFileName))
            {
                // If already computed reload it.
                ALICEVISION_COUT("Voxels array already computed, reload from file: " << voxelsArrayFileName);
                voxelsArray = loadArrayFromFile<Point3d>(voxelsArrayFileName);
            }
            else
            {
                ALICEVISION_COUT("Compute voxels array");
                meshConstruction::ReconstructionPlan rp(lsbase.dimensions, &lsbase.space[0], lsbase.mp, lsbase.pc, lsbase.spaceVoxelsFolderName);
                voxelsArray = rp.computeReconstructionPlanBinSearch(maxPts);
                saveArrayToFile<Point3d>(voxelsArrayFileName, voxelsArray);
            }
            meshConstruction::reconstructSpaceAccordingToVoxelsArray(voxelsArrayFileName, &lsbase, true);
            // Join meshes
            mesh::Mesh* mesh = meshConstruction::joinMeshes(voxelsArrayFileName, &lsbase);

            ALICEVISION_COUT("Saving joined meshes");

            bfs::path spaceBinFileName = outDirectory/"denseReconstruction.bin";
            mesh->saveToBin(spaceBinFileName.string());

            // Export joined mesh to obj
            mesh->saveToObj(outputMesh);

            delete mesh;

            // Join ptsCams
            StaticVector<StaticVector<int>*>* ptsCams = meshConstruction::loadLargeScalePtsCams(lsbase.getRecsDirs(voxelsArray));
            saveArrayOfArraysToFile<int>((outDirectory/"meshPtsCamsFromDGC.bin").string(), ptsCams);
            deleteArrayOfArrays<int>(&ptsCams);
        }
        break;
        case eSingleBlock:
        {
            ALICEVISION_COUT("--- meshing partitioning: single block");
            meshConstruction::LargeScale ls0(&mp, &pc, tmpDirectory.string() + "/");
            ls0.generateSpace(maxPtsPerVoxel, ocTreeDim);
            unsigned long ntracks = std::numeric_limits<unsigned long>::max();
            while(ntracks > maxPts)
            {
                bfs::path dirName = outDirectory/("LargeScaleMaxPts" + common::num2strFourDecimal(ocTreeDim));
                meshConstruction::LargeScale* ls = ls0.cloneSpaceIfDoesNotExists(ocTreeDim, dirName.string() + "/");
                meshConstruction::VoxelsGrid vg(ls->dimensions, &ls->space[0], ls->mp, ls->pc, ls->spaceVoxelsFolderName);
                ntracks = vg.getNTracks();
                delete ls;
                ALICEVISION_COUT("Number of track candidates: " << ntracks);
                if(ntracks > maxPts)
                {
                    ALICEVISION_COUT("ocTreeDim: " << ocTreeDim);
                    double t = (double)ntracks / (double)maxPts;
                    ALICEVISION_COUT("downsample: " << ((t < 2.0) ? "slow" : "fast"));
                    ocTreeDim = (t < 2.0) ? ocTreeDim-100 : ocTreeDim*0.5;
                }
            }
            ALICEVISION_COUT("Number of tracks: " << ntracks);
            ALICEVISION_COUT("ocTreeDim: " << ocTreeDim);
            bfs::path dirName = outDirectory/("LargeScaleMaxPts" + common::num2strFourDecimal(ocTreeDim));
            meshConstruction::LargeScale lsbase(&mp, &pc, dirName.string()+"/");
            lsbase.loadSpaceFromFile();
            meshConstruction::ReconstructionPlan rp(lsbase.dimensions, &lsbase.space[0], lsbase.mp, lsbase.pc, lsbase.spaceVoxelsFolderName);
            StaticVector<int> voxelNeighs(rp.voxels->size() / 8);
            for(int i = 0; i < rp.voxels->size() / 8; i++)
                voxelNeighs.push_back(i);
            meshConstruction::DelaunayGraphCut delaunayGC(lsbase.mp, lsbase.pc);
            StaticVector<Point3d>* hexahsToExcludeFromResultingMesh = nullptr;
            Point3d* hexah = &lsbase.space[0];
            delaunayGC.reconstructVoxel(hexah, &voxelNeighs, outDirectory.string()+"/", lsbase.getSpaceCamsTracksDir(), false, hexahsToExcludeFromResultingMesh,
                                  (meshConstruction::VoxelsGrid*)&rp, lsbase.getSpaceSteps());

            delaunayGC.graphCutPostProcessing();

            // Save mesh as .bin and .obj
            mesh::Mesh* mesh = delaunayGC.createMesh();
            if(mesh->pts->empty())
              throw std::runtime_error("Empty mesh");

            StaticVector<StaticVector<int>*>* ptsCams = delaunayGC.createPtsCams();
            StaticVector<int> usedCams = delaunayGC.getSortedUsedCams();

            mesh::meshPostProcessing(mesh, ptsCams, usedCams, mp, pc, outDirectory.string()+"/", hexahsToExcludeFromResultingMesh, hexah);
            mesh->saveToBin((outDirectory/"denseReconstruction.bin").string());

            saveArrayOfArraysToFile<int>((outDirectory/"meshPtsCamsFromDGC.bin").string(), ptsCams);
            deleteArrayOfArrays<int>(&ptsCams);

            mesh->saveToObj(outputMesh);

            delete mesh;
        }
        break;
        case eUndefined:
        default:
            throw std::invalid_argument("Partitioning not defined");
    }

    common::printfElapsedTime(startTime, "#");
    return EXIT_SUCCESS;
}
