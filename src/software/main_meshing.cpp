// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/delaunaycut/mv_delaunay_GC.hpp>
#include <aliceVision/delaunaycut/mv_delaunay_meshSmooth.hpp>
#include <aliceVision/largeScale/reconstructionPlan.hpp>
#include <aliceVision/planeSweeping/ps_refine_rc.hpp>
#include <aliceVision/CUDAInterfaces/refine.hpp>
#include <aliceVision/structures/mv_filesio.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>


namespace bfs = boost::filesystem;
namespace po = boost::program_options;

#define ALICEVISION_COUT(x) std::cout << x << std::endl
#define ALICEVISION_CERR(x) std::cerr << x << std::endl


enum EMeshingMode {
    eUndefined = 0,
    eLimited = 1,
    eLargeScale = 2
};

EMeshingMode EMeshingMode_stringToEnum(const std::string& s)
{
    if(s == "limited")
        return eLimited;
    if(s == "largeScale")
        return eLargeScale;
    return eUndefined;
}

inline std::istream& operator>>(std::istream& in, EMeshingMode& mode)
{
    std::string s;
    in >> s;
    mode = EMeshingMode_stringToEnum(s);
    return in;
}


int main(int argc, char* argv[])
{
    long startTime = clock();

    std::string iniFilepath;
    std::string outputMesh;
    EMeshingMode meshingMode = eLimited;
    po::options_description inputParams;

    inputParams.add_options()
        ("ini", po::value<std::string>(&iniFilepath)->required(),
            "Configuration file (mvs.ini).")
        ("output", po::value<std::string>(&outputMesh)->required(),
            "Output mesh (OBJ file format).")
        ("mode", po::value<EMeshingMode>(&meshingMode)->default_value(meshingMode),
            "Meshing mode: limited or largeScale.");
    po::variables_map vm;

    try
    {
      po::store(po::parse_command_line(argc, argv, inputParams), vm);

      if(vm.count("help") || (argc == 1))
      {
        ALICEVISION_COUT(inputParams);
        return EXIT_SUCCESS;
      }

      po::notify(vm);
    }
    catch(boost::program_options::required_option& e)
    {
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << inputParams);
      return EXIT_FAILURE;
    }
    catch(boost::program_options::error& e)
    {
      ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
      ALICEVISION_COUT("Usage:\n\n" << inputParams);
      return EXIT_FAILURE;
    }

    ALICEVISION_COUT("ini file: " << iniFilepath);

    // .ini parsing
    multiviewInputParams mip(iniFilepath);
    const double simThr = mip._ini.get<double>("global.simThr", 0.0);
    multiviewParams mp(mip.getNbCameras(), &mip, (float) simThr);
    mv_prematch_cams pc(&mp);

    // .ini parsing
    const int maxPts = mip._ini.get<int>("largeScale.planMaxPts", 30000000);
    const int maxPtsPerVoxel = std::max(maxPts, mip._ini.get<int>("largeScale.planMaxPtsPerVoxel", 30000000));
    int ocTreeDim = mip._ini.get<int>("largeScale.gridLevel0", 1024);
    const auto baseDir = mip._ini.get<std::string>("largeScale.baseDirName", "root01024");

    bfs::path outDirectory = bfs::path(outputMesh).parent_path();
    if(!bfs::is_directory(outDirectory))
        bfs::create_directory(outDirectory);

    switch(meshingMode)
    {
        case eLargeScale:
        {
            ALICEVISION_COUT("--- meshing (large scale)");
            largeScale lsbase(&mp, &pc, mip.mvDir + baseDir + "/");
            lsbase.generateSpace(maxPtsPerVoxel, ocTreeDim);
            std::string voxelsArrayFileName = lsbase.spaceFolderName + "hexahsToReconstruct.bin";
            staticVector<point3d>* voxelsArray = nullptr;
            if(FileExists(voxelsArrayFileName))
            {
                // If already computed reload it.
                ALICEVISION_COUT("Voxels array already computed, reload from file: " << voxelsArrayFileName);
                voxelsArray = loadArrayFromFile<point3d>(voxelsArrayFileName);
            }
            else
            {
                ALICEVISION_COUT("Compute voxels array");
                reconstructionPlan rp(lsbase.dimensions, &lsbase.space[0], lsbase.mp, lsbase.pc, lsbase.spaceVoxelsFolderName);
                voxelsArray = rp.computeReconstructionPlanBinSearch(maxPts);
                saveArrayToFile<point3d>(voxelsArrayFileName, voxelsArray);
            }
            reconstructSpaceAccordingToVoxelsArray(voxelsArrayFileName, &lsbase, true);
            // Join meshes
            mv_mesh* mesh = joinMeshes(voxelsArrayFileName, &lsbase);

            ALICEVISION_COUT("Saving joined meshes");

            std::string spaceBinFileName = mip.mvDir + "mesh.bin";
            mesh->saveToBin(spaceBinFileName);

            // Export joined mesh to obj
            mv_output3D o3d(lsbase.mp);
            o3d.saveMvMeshToObj(mesh, outputMesh);

            delete mesh;

            // Join ptsCams
            staticVector<staticVector<int>*>* ptsCams = loadLargeScalePtsCams(lsbase.getRecsDirs(voxelsArray));
            saveArrayOfArraysToFile<int>(mip.mvDir + "meshPtsCamsFromDGC.bin", ptsCams);
            deleteArrayOfArrays<int>(&ptsCams);
        }
        case eLimited:
        {
            ALICEVISION_COUT("--- meshing (limited scale)");
            largeScale ls0(&mp, &pc, mip.mvDir + baseDir + "/");
            ls0.generateSpace(maxPtsPerVoxel, ocTreeDim);
            unsigned long ntracks = std::numeric_limits<unsigned long>::max();
            while(ntracks > maxPts)
            {
                std::string dirName = mip.mvDir + "largeScaleMaxPts" + num2strFourDecimal(ocTreeDim) + "/";
                largeScale* ls = ls0.cloneSpaceIfDoesNotExists(ocTreeDim, dirName);
                voxelsGrid vg(ls->dimensions, &ls->space[0], ls->mp, ls->pc, ls->spaceVoxelsFolderName);
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
            largeScale lsbase(&mp, &pc, mip.mvDir + "largeScaleMaxPts" + num2strFourDecimal(ocTreeDim) + "/");
            lsbase.loadSpaceFromFile();
            reconstructionPlan rp(lsbase.dimensions, &lsbase.space[0], lsbase.mp, lsbase.pc, lsbase.spaceVoxelsFolderName);
            staticVector<int> voxelNeighs(rp.voxels->size() / 8);
            for(int i = 0; i < rp.voxels->size() / 8; i++)
                voxelNeighs.push_back(i);
            mv_delaunay_GC delaunayGC(lsbase.mp, lsbase.pc);
            staticVector<point3d>* hexahsToExcludeFromResultingMesh = nullptr;
            point3d* hexah = &lsbase.space[0];
            delaunayGC.reconstructVoxel(hexah, &voxelNeighs, mip.mvDir, lsbase.getSpaceCamsTracksDir(), false, hexahsToExcludeFromResultingMesh,
                                  (voxelsGrid*)&rp, lsbase.getSpaceSteps());

            bool exportDebugGC = (float)mip._ini.get<bool>("delaunaycut.exportDebugGC", false);
            //if(exportDebugGC)
            //    delaunayGC.saveMeshColoredByCamsConsistency(mip.mvDir + "meshColoredbyCamsConsistency.wrl", mip.mvDir + "meshColoredByVisibility.wrl");

            delaunayGC.graphCutPostProcessing();
            if(exportDebugGC)
                delaunayGC.saveMeshColoredByCamsConsistency(mip.mvDir + "meshColoredbyCamsConsistency_postprocess.wrl", mip.mvDir + "meshColoredByVisibility_postprocess.wrl");

            // Save mesh as .bin and .obj
            mv_mesh* mesh = delaunayGC.createMesh();
            staticVector<staticVector<int>*>* ptsCams = delaunayGC.createPtsCams();
            staticVector<int> usedCams = delaunayGC.getSortedUsedCams();

            meshPostProcessing(mesh, ptsCams, usedCams, mp, pc, mip.mvDir, hexahsToExcludeFromResultingMesh, hexah);
            mesh->saveToBin(mip.mvDir + "mesh.bin");

            saveArrayOfArraysToFile<int>(mip.mvDir + "meshPtsCamsFromDGC.bin", ptsCams);
            deleteArrayOfArrays<int>(&ptsCams);

            mv_output3D o3d(&mp);
            o3d.saveMvMeshToObj(mesh, outputMesh);

            delete mesh;
        }
    }

    printfElapsedTime(startTime, "#");
    return EXIT_SUCCESS;
}
