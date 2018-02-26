// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/PreMatchCams.hpp>
#include <aliceVision/mesh/meshPostProcessing.hpp>
#include <aliceVision/fuseCut/LargeScale.hpp>
#include <aliceVision/fuseCut/ReconstructionPlan.hpp>
#include <aliceVision/fuseCut/DelaunayGraphCut.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

using namespace aliceVision;
namespace bfs = boost::filesystem;
namespace po = boost::program_options;

enum EPartitioning {
    eUndefined = 0,
    eSingleBlockVoxels = 1,
    eSingleBlock = 2,
    eAuto = 3,
};

EPartitioning EPartitioning_stringToEnum(const std::string& s)
{
    if(s == "singleBlockVoxels")
        return eSingleBlockVoxels;
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
    system::Timer timer;

    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string iniFilepath;
    std::string outputMesh;
    std::string depthMapFolder;
    std::string depthMapFilterFolder;

    EPartitioning partitioning = eSingleBlock;
    po::options_description inputParams;
    int maxPts = 6000000;
    int maxPtsPerVoxel = 6000000;
    int smoothingIteration = 0;
    float smoothingWeight = 1.0;

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
        ("smoothingIteration", po::value<int>(&smoothingIteration)->default_value(smoothingIteration),
            "Meshing post-processing: Number of smoothing iteration.")
        ("smoothingWeight", po::value<float>(&smoothingWeight)->default_value(smoothingWeight),
            "Meshing post-processing: smoothing weight.")
        ("partitioning", po::value<EPartitioning>(&partitioning)->default_value(partitioning),
            "Partitioning: 'singleBlock', 'singleBlockVoxels' or 'auto'.");

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

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    // .ini and files parsing
    mvsUtils::MultiViewParams mp(iniFilepath, depthMapFolder, depthMapFilterFolder, true);
    mvsUtils::PreMatchCams pc(&mp);

    int ocTreeDim = mp._ini.get<int>("LargeScale.gridLevel0", 1024);
    const auto baseDir = mp._ini.get<std::string>("LargeScale.baseDirName", "root01024");

    // semiGlobalMatching
    mp._ini.put("meshEnergyOpt.smoothNbIterations", smoothingIteration);
    mp._ini.put("meshEnergyOpt.lambda", smoothingWeight);

    bfs::path outDirectory = bfs::path(outputMesh).parent_path();
    if(!bfs::is_directory(outDirectory))
        bfs::create_directory(outDirectory);

    bfs::path tmpDirectory = outDirectory / "tmp";

    switch(partitioning)
    {
        case eAuto:
        {
            ALICEVISION_LOG_INFO("Meshing partitioning mode: auto.");
            fuseCut::LargeScale lsbase(&mp, &pc, tmpDirectory.string() + "/");
            lsbase.generateSpace(maxPtsPerVoxel, ocTreeDim);
            std::string voxelsArrayFileName = lsbase.spaceFolderName + "hexahsToReconstruct.bin";
            StaticVector<Point3d>* voxelsArray = nullptr;
            if(bfs::exists(voxelsArrayFileName))
            {
                // If already computed reload it.
                ALICEVISION_LOG_INFO("Voxels array already computed, reload from file: " << voxelsArrayFileName);
                voxelsArray = loadArrayFromFile<Point3d>(voxelsArrayFileName);
            }
            else
            {
                ALICEVISION_LOG_INFO("Compute voxels array.");
                fuseCut::ReconstructionPlan rp(lsbase.dimensions, &lsbase.space[0], lsbase.mp, lsbase.pc, lsbase.spaceVoxelsFolderName);
                voxelsArray = rp.computeReconstructionPlanBinSearch(maxPts);
                saveArrayToFile<Point3d>(voxelsArrayFileName, voxelsArray);
            }
            fuseCut::reconstructSpaceAccordingToVoxelsArray(voxelsArrayFileName, &lsbase, true);
            // Join meshes
            mesh::Mesh* mesh = fuseCut::joinMeshes(voxelsArrayFileName, &lsbase);

            if(mesh->pts->empty() || mesh->tris->empty())
              throw std::runtime_error("Empty mesh");

            ALICEVISION_LOG_INFO("Saving joined meshes...");

            bfs::path spaceBinFileName = outDirectory/"denseReconstruction.bin";
            mesh->saveToBin(spaceBinFileName.string());

            // Export joined mesh to obj
            mesh->saveToObj(outputMesh);

            delete mesh;

            // Join ptsCams
            StaticVector<StaticVector<int>*>* ptsCams = fuseCut::loadLargeScalePtsCams(lsbase.getRecsDirs(voxelsArray));
            saveArrayOfArraysToFile<int>((outDirectory/"meshPtsCamsFromDGC.bin").string(), ptsCams);
            deleteArrayOfArrays<int>(&ptsCams);
        }
        break;
        case eSingleBlockVoxels:
        {
            ALICEVISION_LOG_INFO("Meshing partitioning mode: single block.");
            fuseCut::LargeScale ls0(&mp, &pc, tmpDirectory.string() + "/");
            ls0.generateSpace(maxPtsPerVoxel, ocTreeDim);
            unsigned long ntracks = std::numeric_limits<unsigned long>::max();
            while(ntracks > maxPts)
            {
                bfs::path dirName = outDirectory/("LargeScaleMaxPts" + mvsUtils::num2strFourDecimal(ocTreeDim));
                fuseCut::LargeScale* ls = ls0.cloneSpaceIfDoesNotExists(ocTreeDim, dirName.string() + "/");
                fuseCut::VoxelsGrid vg(ls->dimensions, &ls->space[0], ls->mp, ls->pc, ls->spaceVoxelsFolderName);
                ntracks = vg.getNTracks();
                delete ls;
                ALICEVISION_LOG_INFO("Number of track candidates: " << ntracks);
                if(ntracks > maxPts)
                {
                    ALICEVISION_LOG_INFO("ocTreeDim: " << ocTreeDim);
                    double t = (double)ntracks / (double)maxPts;
                    ALICEVISION_LOG_INFO("downsample: " << ((t < 2.0) ? "slow" : "fast"));
                    ocTreeDim = (t < 2.0) ? ocTreeDim-100 : ocTreeDim*0.5;
                }
            }
            ALICEVISION_LOG_INFO("Number of tracks: " << ntracks);
            ALICEVISION_LOG_INFO("ocTreeDim: " << ocTreeDim);
            bfs::path dirName = outDirectory/("LargeScaleMaxPts" + mvsUtils::num2strFourDecimal(ocTreeDim));
            fuseCut::LargeScale lsbase(&mp, &pc, dirName.string()+"/");
            lsbase.loadSpaceFromFile();
            fuseCut::ReconstructionPlan rp(lsbase.dimensions, &lsbase.space[0], lsbase.mp, lsbase.pc, lsbase.spaceVoxelsFolderName);

            StaticVector<int> voxelNeighs;
            voxelNeighs.resize(rp.voxels->size() / 8);
            ALICEVISION_LOG_INFO("voxelNeighs.size(): " << voxelNeighs.size());
            for(int i = 0; i < voxelNeighs.size(); ++i)
                voxelNeighs[i] = i;

            fuseCut::DelaunayGraphCut delaunayGC(lsbase.mp, lsbase.pc);
            Point3d* hexah = &lsbase.space[0];
            delaunayGC.reconstructVoxel(hexah, &voxelNeighs, outDirectory.string()+"/", lsbase.getSpaceCamsTracksDir(), false,
                                  (fuseCut::VoxelsGrid*)&rp, lsbase.getSpaceSteps(), 0);

            delaunayGC.graphCutPostProcessing();

            // Save mesh as .bin and .obj
            mesh::Mesh* mesh = delaunayGC.createMesh();
            if(mesh->pts->empty() || mesh->tris->empty())
              throw std::runtime_error("Empty mesh");

            StaticVector<StaticVector<int>*>* ptsCams = delaunayGC.createPtsCams();
            StaticVector<int> usedCams = delaunayGC.getSortedUsedCams();

            StaticVector<Point3d>* hexahsToExcludeFromResultingMesh = nullptr;
            mesh::meshPostProcessing(mesh, ptsCams, usedCams, mp, pc, outDirectory.string()+"/", hexahsToExcludeFromResultingMesh, hexah);
            mesh->saveToBin((outDirectory/"denseReconstruction.bin").string());

            saveArrayOfArraysToFile<int>((outDirectory/"meshPtsCamsFromDGC.bin").string(), ptsCams);
            deleteArrayOfArrays<int>(&ptsCams);

            mesh->saveToObj(outputMesh);

            delete mesh;
        }
        break;
        case eSingleBlock:
        {
            fuseCut::DelaunayGraphCut delaunayGC(&mp, &pc);
            std::array<Point3d, 8> hexah;
            
            float minPixSize;
            fuseCut::Fuser fs(&mp, &pc);
            fs.divideSpace(&hexah[0], minPixSize);
            Voxel dimensions = fs.estimateDimensions(&hexah[0], &hexah[0], 0, ocTreeDim);
            StaticVector<Point3d>* voxels = mvsUtils::computeVoxels(&hexah[0], dimensions);

            StaticVector<int> voxelNeighs;
            voxelNeighs.resize(voxels->size() / 8);
            ALICEVISION_LOG_INFO("voxelNeighs.size(): " << voxelNeighs.size());
            for(int i = 0; i < voxelNeighs.size(); ++i)
                voxelNeighs[i] = i;
            Point3d spaceSteps;
            {
                Point3d vx = hexah[1] - hexah[0];
                Point3d vy = hexah[3] - hexah[0];
                Point3d vz = hexah[4] - hexah[0];
                spaceSteps.x = (vx.size() / (double)dimensions.x) / (double)ocTreeDim;
                spaceSteps.y = (vy.size() / (double)dimensions.y) / (double)ocTreeDim;
                spaceSteps.z = (vz.size() / (double)dimensions.z) / (double)ocTreeDim;
            }
            delaunayGC.reconstructVoxel(&hexah[0], &voxelNeighs, outDirectory.string()+"/", outDirectory.string()+"/SpaceCamsTracks/", false,
                                        nullptr, spaceSteps, maxPts);
            // TODO change the function name: reconstructFromDepthMaps(hexah);

            delaunayGC.graphCutPostProcessing();

            // Save mesh as .bin and .obj
            mesh::Mesh* mesh = delaunayGC.createMesh();
            if(mesh->pts->empty() || mesh->tris->empty())
              throw std::runtime_error("Empty mesh");

            StaticVector<StaticVector<int>*>* ptsCams = delaunayGC.createPtsCams();
            StaticVector<int> usedCams = delaunayGC.getSortedUsedCams();

            StaticVector<Point3d>* hexahsToExcludeFromResultingMesh = nullptr;
            mesh::meshPostProcessing(mesh, ptsCams, usedCams, mp, pc, outDirectory.string()+"/", hexahsToExcludeFromResultingMesh, &hexah[0]);
            mesh->saveToBin((outDirectory/"denseReconstruction.bin").string());

            saveArrayOfArraysToFile<int>((outDirectory/"meshPtsCamsFromDGC.bin").string(), ptsCams);
            deleteArrayOfArrays<int>(&ptsCams);
            delete voxels;

            mesh->saveToObj(outputMesh);

            delete mesh;
            break;
        }
        case eUndefined:
        default:
            throw std::invalid_argument("Partitioning not defined");
    }

    ALICEVISION_LOG_INFO("Task done in (s): " + std::to_string(timer.elapsed()));
    return EXIT_SUCCESS;
}
