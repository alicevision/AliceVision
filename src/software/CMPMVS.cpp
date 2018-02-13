// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/structures/Point3d.hpp>
#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/delaunaycut/mv_delaunay_GC.hpp>
#include <aliceVision/delaunaycut/mv_delaunay_meshSmooth.hpp>
#include <aliceVision/largeScale/ReconstructionPlan.hpp>
#include <aliceVision/depthMap/RefineRc.hpp>
#include <aliceVision/common/fileIO.hpp>
#include <aliceVision/mesh/Retexturer.hpp>

#include <boost/filesystem.hpp>

#include <bitset>

using namespace std;
namespace bfs = boost::filesystem;


struct CommandLine
{
    enum Step
    {
        COMPUTE_CAM_PAIRS,
        CREATE_DEPTHMAP,
        FILTER_DEPTHMAP,
        MESHING,
        TEXTURING,
        END
    };
    void printHelp()
    {
        cout << R"(
    Usage: cmpmvs INIFILE [OPTION]...
        --computeCamPairs       compute connectivity between camera pairs
        --createDepthmap        enable createDepthmap step
        --filterDepthmap        enable filterDepthmap step
        --meshing [MODE]        enable meshing step using the MODE method
                                    modes: - large (default)
                                           - limited
        --texturing             enable texturing step
        --all                   enable all steps
        --from [STEP]           enable all steps starting from STEP
        --rangeStart [INDEX]    compute only a subset of cameras (to compute multiple parts in parallel)
        --rangeSize [SIZE]      number of cameras to process (from rangeStart to rangeStart + rangeSize)
        --help                  display this help and exit

    Examples:
        cmpmvs parameters.ini --computeCamPairs --createDepthmap
        cmpmvs parameters.ini --meshing limited
        cmpmvs parameters.ini --all
        cmpmvs parameters.ini --from filterDepthmap
        )" << endl;
    }
    bool isValid()
    {
        if(meshingMode != "large" && meshingMode != "limited")
        {
            cerr << "ERROR: unknown meshing mode" << endl;
            return false;
        }
        if(iniFile.empty())
        {
            cerr << "ERROR: missing .ini file" << endl;
            return false;
        }
        if(!unrecognized.empty())
        {
            for(auto& arg : unrecognized)
                cerr << "ERROR: unrecognized argument: " << arg << endl;
            return false;
        }
        if(steps.none())
        {
            cerr << "ERROR: nothing to do" << endl;
            return false;
        }
        return true;
    }
    bool getArg(int argc, char* argv[], int i, string& value) const
    {
        if(i>=argc) return false;
        if(*argv[i] == '-') return false;
        value = argv[i];
        return true;
    }
    bool getArg(int argc, char* argv[], int i, int& value) const
    {
        if(i>=argc) return false;
        if(*argv[i] == '-') return false;
        string s = argv[i];
        value = stoi(s);
        return true;
    }
    bool matches(const string& option, const string& key) const
    {
        return option == key;
    }
    bool parse(int argc, char* argv[])
    {
        auto stepFromString = [&](const string& value)
        {
            if(value == "computeCamPairs")
                return Step::COMPUTE_CAM_PAIRS;
            if(value == "createDepthmap")
                return Step::CREATE_DEPTHMAP;
            if(value == "filterDepthmap")
                return Step::FILTER_DEPTHMAP;
            if(value == "meshing")
                return Step::MESHING;
            if(value == "texturing")
                return Step::TEXTURING;
            cerr << "ERROR: unknown step " << value << endl;
            return Step::END;
        };
        for(int i = 1; i < argc; ++i)
        {
            if(*argv[i] != '-')
            {
                if(!iniFile.empty())
                {
                    cerr << "ERROR: doesn't support multiple INI files in input" << endl;
                    return false;
                }
                getArg(argc, argv, i, iniFile);
            }
            else
            {
                if(matches(argv[i], "--all"))
                    steps.set();
                else if(matches(argv[i], "--computeCamPairs"))
                  steps.set(Step::COMPUTE_CAM_PAIRS);
                else if(matches(argv[i], "--createDepthmap"))
                    steps.set(Step::CREATE_DEPTHMAP);
                else if(matches(argv[i], "--filterDepthmap"))
                    steps.set(Step::FILTER_DEPTHMAP);
                else if(matches(argv[i], "--meshing"))
                {
                    if(!getArg(argc, argv, ++i, meshingMode))
                        --i; // argument is optional
                    steps.set(Step::MESHING);
                }
                else if(matches(argv[i], "--texturing"))
                    steps.set(Step::TEXTURING);
                else if(matches(argv[i], "--from"))
                {
                    string arg;
                    getArg(argc, argv, ++i, arg);
                    Step startStep = stepFromString(arg);
                    if(startStep == Step::END)
                        return false;
                    for(size_t i = startStep; i<steps.size()-1; ++i)
                        steps.set(static_cast<Step>(i));
                }
                else if(matches(argv[i], "--rangeStart"))
                {
                    getArg(argc, argv, ++i, rangeStart);
                }
                else if(matches(argv[i], "--rangeSize"))
                {
                    getArg(argc, argv, ++i, rangeSize);
                }
                else if(matches(argv[i], "--help"))
                    return false;
                else
                    unrecognized.emplace_back(argv[i]);
            }
        }
        return isValid();
    }
    string iniFile;
    string meshingMode = "large";
    int rangeStart = -1;
    int rangeSize = -1;
    bitset<Step::END> steps;
    vector<string> unrecognized;
};


bool checkHardwareCompatibility()
{
    if(listCUDADevices(false) < 1)
    {
        cerr << "ERROR: no CUDA capable devices were detected." << endl;
        return false;
    }
    return true;
}

int main(int argc, char* argv[])
{
    long startTime = clock();

    // check hardware compatibility
    if(!checkHardwareCompatibility())
        return EXIT_FAILURE;

    // command line parsing
    CommandLine cmdline;
    if(!cmdline.parse(argc, argv))
    {
        cmdline.printHelp();
        return EXIT_FAILURE;
    }

    // .ini parsing
    const auto rootPath = bfs::path(cmdline.iniFile).parent_path();
    const auto depthMapFolder = rootPath / "depthMap";
    const auto depthMapFilterFolder = rootPath / "depthMapFilter";
    bfs::create_directory(depthMapFolder);
    bfs::create_directory(depthMapFilterFolder);
    multiviewInputParams mip(cmdline.iniFile, depthMapFolder.string(), depthMapFilterFolder.string());
    const double simThr = mip._ini.get<double>("global.simThr", 0.0);
    const int minNumOfConsistensCams = mip._ini.get<int>("filter.minNumOfConsistentCams", 3);
    const int maxPts = mip._ini.get<int>("LargeScale.planMaxPts", 30000000);
    const int maxPtsPerVoxel = std::max(maxPts, mip._ini.get<int>("LargeScale.planMaxPtsPerVoxel", 30000000));
    int ocTreeDim = mip._ini.get<int>("LargeScale.gridLevel0", 1024);
    const auto baseDir = mip._ini.get<std::string>("LargeScale.baseDirName", "root01024");

    // build out path
    bfs::path outPath(mip._ini.get<std::string>("global.outDir", "_OUT"));
    if(outPath.is_relative())
        outPath = bfs::path(mip.mvDir) / outPath;
    outPath += "/";
    mip.outDir = outPath.string();

    multiviewParams mp(mip.getNbCameras(), &mip, (float) simThr);
    mv_prematch_cams pc(&mp);
    StaticVector<int> cams(mp.ncams);
    if(cmdline.rangeSize == -1)
    {
        for(int rc = 0; rc < mp.ncams; rc++) // process all cameras
            cams.push_back(rc);
    }
    else
    {
        if(cmdline.rangeStart < 0)
        {
            cerr << "invalid subrange of cameras to process." << endl;
            return EXIT_FAILURE;
        }
        for(int rc = cmdline.rangeStart; rc < std::min(cmdline.rangeStart + cmdline.rangeSize, mp.ncams); ++rc)
            cams.push_back(rc);
        if(cams.empty())
        {
            cout << "No camera to process" << endl;
            return EXIT_SUCCESS;
        }
    }

    if(cmdline.steps.test(CommandLine::Step::COMPUTE_CAM_PAIRS))
    {
        std::cout << "--- compute camera pairs" << std::endl;
        pc.precomputeIncidentMatrixCamsFromSeeds();
    }
    
    if(cmdline.steps.test(CommandLine::Step::CREATE_DEPTHMAP))
    {
        cout << "--- create depthmap" << endl;
        computeDepthMapsPSSGM(&mp, &pc, cams);
        refineDepthMaps(&mp, &pc, cams);
    }

    if(cmdline.steps.test(CommandLine::Step::FILTER_DEPTHMAP))
    {
        cout << "--- filter depthmap" << endl;
        Fuser fs(&mp, &pc);
        fs.filterGroups(cams, 0, 0, 10);
        fs.filterDepthMaps(cams, minNumOfConsistensCams, 4); //minNumOfConsistensCamsWithLowSimilarity = 4
    }

    if(cmdline.steps.test(CommandLine::Step::MESHING))
    {
        bfs::create_directory(outPath);

        if(cmdline.meshingMode == "large")
        {
            cout << "--- meshing (large scale)" << endl;
            LargeScale lsbase(&mp, &pc, mip.mvDir + baseDir + "/");
            lsbase.generateSpace(maxPtsPerVoxel, ocTreeDim);
            string voxelsArrayFileName = lsbase.spaceFolderName + "hexahsToReconstruct.bin";
            StaticVector<Point3d>* voxelsArray = nullptr;
            if(FileExists(voxelsArrayFileName))
            {
                // If already computed reload it.
                std::cout << "Voxels array already computed, reload from file: " << voxelsArrayFileName << std::endl;
                voxelsArray = loadArrayFromFile<Point3d>(voxelsArrayFileName);
            }
            else
            {
                std::cout << "Compute voxels array" << std::endl;
                ReconstructionPlan rp(lsbase.dimensions, &lsbase.space[0], lsbase.mp, lsbase.pc, lsbase.spaceVoxelsFolderName);
                voxelsArray = rp.computeReconstructionPlanBinSearch(maxPts);
                saveArrayToFile<Point3d>(voxelsArrayFileName, voxelsArray);
            }
            reconstructSpaceAccordingToVoxelsArray(voxelsArrayFileName, &lsbase, true);
            // Join meshes
            Mesh* mesh = joinMeshes(voxelsArrayFileName, &lsbase);

            std::cout << "Saving joined meshes" << std::endl;

            std::string spaceBinFileName = mip.mvDir + "mesh.bin";
            mesh->saveToBin(spaceBinFileName);

            // Export joined mesh to obj
            mesh->saveToObj((outPath / "mesh.obj").string());

            delete mesh;

            // Join ptsCams
            StaticVector<StaticVector<int>*>* ptsCams = loadLargeScalePtsCams(lsbase.getRecsDirs(voxelsArray));
            saveArrayOfArraysToFile<int>(mip.mvDir + "meshPtsCamsFromDGC.bin", ptsCams);
            deleteArrayOfArrays<int>(&ptsCams);
        }

        if(cmdline.meshingMode == "limited")
        {
            cout << "--- meshing (limited scale)" << endl;
            LargeScale ls0(&mp, &pc, mip.mvDir + baseDir + "/");
            ls0.generateSpace(maxPtsPerVoxel, ocTreeDim);
            unsigned long ntracks = std::numeric_limits<unsigned long>::max();
            while(ntracks > maxPts)
            {
                string dirName = mip.mvDir + "LargeScaleMaxPts" + num2strFourDecimal(ocTreeDim) + "/";
                LargeScale* ls = ls0.cloneSpaceIfDoesNotExists(ocTreeDim, dirName);
                VoxelsGrid vg(ls->dimensions, &ls->space[0], ls->mp, ls->pc, ls->spaceVoxelsFolderName);
                ntracks = vg.getNTracks();
                delete ls;
                if(ntracks > maxPts)
                {
                    double t = (double)ntracks / (double)maxPts;
                    ocTreeDim = (t < 2.0) ? ocTreeDim-100 : ocTreeDim*0.5;
                }
            }
            LargeScale lsbase(&mp, &pc, mip.mvDir + "LargeScaleMaxPts" + num2strFourDecimal(ocTreeDim) + "/");
            lsbase.loadSpaceFromFile();
            ReconstructionPlan rp(lsbase.dimensions, &lsbase.space[0], lsbase.mp, lsbase.pc, lsbase.spaceVoxelsFolderName);
            StaticVector<int> voxelNeighs(rp.voxels->size() / 8);
            for(int i = 0; i < rp.voxels->size() / 8; i++)
                voxelNeighs.push_back(i);
            mv_delaunay_GC delaunayGC(lsbase.mp, lsbase.pc);
            StaticVector<Point3d>* hexahsToExcludeFromResultingMesh = nullptr;
            Point3d* hexah = &lsbase.space[0];
            delaunayGC.reconstructVoxel(hexah, &voxelNeighs, mip.mvDir, lsbase.getSpaceCamsTracksDir(), false, hexahsToExcludeFromResultingMesh,
                                  (VoxelsGrid*)&rp, lsbase.getSpaceSteps());

            delaunayGC.graphCutPostProcessing();

            // Save mesh as .bin and .obj
            Mesh* mesh = delaunayGC.createMesh();
            StaticVector<StaticVector<int>*>* ptsCams = delaunayGC.createPtsCams();
            StaticVector<int> usedCams = delaunayGC.getSortedUsedCams();

            meshPostProcessing(mesh, ptsCams, usedCams, mp, pc, mip.mvDir, hexahsToExcludeFromResultingMesh, hexah);
            mesh->saveToBin(mip.mvDir + "mesh.bin");

            saveArrayOfArraysToFile<int>(mip.mvDir + "meshPtsCamsFromDGC.bin", ptsCams);
            deleteArrayOfArrays<int>(&ptsCams);

            mesh->saveToObj((outPath / "mesh.obj").string());

            delete mesh;
        }
    }


    if(cmdline.steps.test(CommandLine::Step::TEXTURING))
    {
        cout << "--- texturing" << endl;

        // load mesh
        Retexturer texMesh;
        texMesh.me = new Mesh();
        if(!texMesh.me->loadFromBin(mip.mvDir + "mesh.bin"))
        {
            cerr << "Unable to load " << mip.mvDir << "mesh.bin" << endl;
            return EXIT_FAILURE;
        }

        // load cam visibilities per point
        StaticVector<StaticVector<int>*>* ptsCams = loadArrayOfArraysFromFile<int>(mip.mvDir + "meshPtsCamsFromDGC.bin");
        
        // create uv atlas
        texMesh.texParams.textureSide = mp.mip->_ini.get<unsigned int>("uvatlas.texSide", 8192);
        texMesh.texParams.padding = mp.mip->_ini.get<unsigned int>("uvatlas.gutter", 15);
        texMesh.texParams.downscale = mp.mip->_ini.get<unsigned int>("uvatlas.scale", 2);

        auto* updatedPtsCams = texMesh.generateUVs(mp, ptsCams);
        std::swap(ptsCams, updatedPtsCams);
        deleteArrayOfArrays<int>(&updatedPtsCams);
        texMesh.saveAsOBJ(outPath, "texturedMesh");
        texMesh.generateTextures(mp, ptsCams, outPath);
    }

    printfElapsedTime(startTime, "#");

    return EXIT_SUCCESS;
}
