#include "largeScale.hpp"

#include <aliceVision/delaunaycut/mv_delaunay_GC.hpp>
#include <aliceVision/structures/mv_filesio.hpp>

#include <boost/filesystem.hpp>


namespace bfs = boost::filesystem;

largeScale::largeScale(multiviewParams* _mp, mv_prematch_cams* _pc, std::string _spaceFolderName)
  : o3d(_mp)
  , mp(_mp)
  , pc(_pc)
  , spaceFolderName(_spaceFolderName)
  , spaceVoxelsFolderName(_spaceFolderName + "_data/")
  , spaceFileName(spaceFolderName + "/space.txt")
{
    bfs::create_directory(spaceFolderName);
    bfs::create_directory(spaceVoxelsFolderName);

    doVisualize = mp->mip->_ini.get<bool>("largeScale.doVisualizeOctreeTracks", false);
    doVisualizeVoxels = mp->mip->_ini.get<bool>("largeScale.doVisualizeVoxels", false);
}

largeScale::~largeScale()
{
}

bool largeScale::isSpaceSaved()
{
    return FileExists(spaceFileName);
}

void largeScale::saveSpaceToFile()
{
    FILE* f = fopen(spaceFileName.c_str(), "w");
    fprintf(f, "%lf %lf %lf %lf %lf %lf %lf %lf\n", space[0].x, space[1].x, space[2].x, space[3].x, space[4].x,
            space[5].x, space[6].x, space[7].x);
    fprintf(f, "%lf %lf %lf %lf %lf %lf %lf %lf\n", space[0].y, space[1].y, space[2].y, space[3].y, space[4].y,
            space[5].y, space[6].y, space[7].y);
    fprintf(f, "%lf %lf %lf %lf %lf %lf %lf %lf\n", space[0].z, space[1].z, space[2].z, space[3].z, space[4].z,
            space[5].z, space[6].z, space[7].z);
    fprintf(f, "%i %i %i\n", dimensions.x, dimensions.y, dimensions.z);
    fprintf(f, "%i\n", maxOcTreeDim);
    fclose(f);
}

void largeScale::loadSpaceFromFile()
{
    FILE* f = fopen(spaceFileName.c_str(), "r");
    fscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf\n",
           &space[0].x, &space[1].x, &space[2].x, &space[3].x,
           &space[4].x, &space[5].x, &space[6].x, &space[7].x);
    fscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf\n",
           &space[0].y, &space[1].y, &space[2].y, &space[3].y,
           &space[4].y, &space[5].y, &space[6].y, &space[7].y);
    fscanf(f, "%lf %lf %lf %lf %lf %lf %lf %lf\n",
           &space[0].z, &space[1].z, &space[2].z, &space[3].z,
           &space[4].z, &space[5].z, &space[6].z, &space[7].z);
    fscanf(f, "%i %i %i\n", &dimensions.x, &dimensions.y, &dimensions.z);
    fscanf(f, "%i\n", &maxOcTreeDim);
    fclose(f);
}

void largeScale::initialEstimateSpace(int maxOcTreeDim)
{
    float minPixSize;
    mv_fuse* fs = new mv_fuse(mp, pc);
    fs->divideSpace(&space[0], minPixSize);
    dimensions = fs->estimateDimensions(&space[0], &space[0], 0, maxOcTreeDim);
    delete fs;
}

void largeScale::visualizeVoxels()
{
    voxelsGrid* vg = new voxelsGrid(dimensions, &space[0], mp, pc, spaceVoxelsFolderName);

    {
        std::string camerasToWrlFileName = spaceFolderName + "cameras.wrl";
        if(!FileExists(camerasToWrlFileName))
        {
            o3d.writeCamerasToWrl(camerasToWrlFileName, mp, 0, 0.0f);

            camerasToWrlFileName = spaceFolderName + "camerasStreetView.wrl";
            o3d.writeCamerasToWrl(camerasToWrlFileName, mp, 6, 0.0f);

            camerasToWrlFileName = spaceFolderName + "camerasStreetViewUp.wrl";
            o3d.writeCamerasToWrl(camerasToWrlFileName, mp, 6, 100.0f);

            std::string camerasDepthToWrlFileName = spaceFolderName + "camerasDepth.wrl";
            o3d.writeCamerasDepthToWrl(camerasDepthToWrlFileName, mp);
        }
    }

    std::string spaceWrlFileName = spaceFolderName + "space.wrl";

    std::string spaceVoxelsWrlFileName = spaceFolderName + "spaceVoxels.wrl";
    {
        FILE* f = fopen(spaceVoxelsWrlFileName.c_str(), "w");
        fprintf(f, "#VRML V2.0 utf8\n");
        // printf("creating wrl\n");

        for(int i = 0; i < vg->voxels->size() / 8; i++)
        {
            std::string vfn = vg->getVoxelFolderName(i);
            if(FolderExists(vfn))
            {
                o3d.printfHexahedron(&(*vg->voxels)[i * 8], f, mp);
            }
        }
        // printfGroupCameras(f, mp, 0.001);

        fclose(f);
    }

    std::string spacePtsWrlFileName = spaceFolderName + "spacePts.wrl";
    if(!FileExists(spacePtsWrlFileName))
    {
        o3d.savePrematchToWrl(spacePtsWrlFileName, 0, 1);
    }

    FILE* f = fopen(spaceWrlFileName.c_str(), "w");
    fprintf(f, "#VRML V2.0 utf8\n");
    fprintf(f, "Background {\n skyColor 1 1 1 \n } \n");
    fprintf(f, "Inline{ url [\"spaceVoxels.wrl\"] \n }\n");
    fprintf(f, "Inline{ url [\"spaceVoxelsNames.wrl\"] \n }\n");
    fprintf(f, "Inline{ url [\"spacePts.wrl\"] \n }\n");
    fprintf(f, "Inline{ url [\"cameras.wrl\"] \n }\n");
    fclose(f);

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////

    std::string fn = spaceFolderName + "spaceVoxelsNames.wrl";
    f = fopen(fn.c_str(), "w");
    fprintf(f, "#VRML V2.0 utf8\n");
    fprintf(f, "Background {\n skyColor 1 1 1 \n } \n");

    for(int i = 0; i < vg->voxels->size() / 8; i++)
    {
        std::string vfn = vg->getVoxelFolderName(i);
        if(FolderExists(vfn))
        {
            voxel v = vg->getVoxelForId(i);
            point3d voxelcg = point3d(0.0f, 0.0f, 0.0f);
            for(int j = 0; j < 8; j++)
            {
                voxelcg = voxelcg + (*vg->voxels)[i * 8 + j];
            }
            voxelcg = voxelcg / 8.0f;
            float ts = ((*vg->voxels)[i * 8 + 0] - (*vg->voxels)[i * 8 + 1]).size() / 10.0f;

            fprintf(f, "Transform {\n");
            fprintf(f, "\t translation  %f %f %f \n", (float)voxelcg.x, (float)voxelcg.y, (float)voxelcg.z);
            fprintf(f, "\t\t children [\n");
            fprintf(f, "\t\t\t Shape {\n");
            fprintf(f, "\t\t\t\t appearance Appearance {\n");
            fprintf(f, "\t\t\t\t\t material Material { diffuseColor 1 0 0 }\n");
            fprintf(f, "\t\t\t\t }\n");
            fprintf(f, "\t\t\t\t geometry Text{\n");
            std::string text = num2str(i) + "-" + num2str(v.x) + "," + num2str(v.y) + "," + num2str(v.z);
            fprintf(f, "\t\t\t\t\t string [ \"%s\" ]\n", text.c_str());
            fprintf(f, "\t\t\t\t\t fontStyle FontStyle {\n");
            fprintf(f, "\t\t\t\t\t\t family  \"SANS\"\n");
            fprintf(f, "\t\t\t\t\t\t style   \"BOLD\"\n");
            fprintf(f, "\t\t\t\t\t\t size    %f\n", ts);
            fprintf(f, "\t\t\t\t\t\t justify \"MIDDLE\"\n");
            fprintf(f, "\t\t\t\t\t }\n");
            fprintf(f, "\t\t\t\t }\n");
            fprintf(f, "\t\t\t }\n");
            fprintf(f, "\t\t ]\n");
            fprintf(f, " }\n");
        }
    }

    fclose(f);

    delete vg;
}

std::string largeScale::getSpaceCamsTracksDir()
{
    voxelsGrid* vg = new voxelsGrid(dimensions, &space[0], mp, pc, spaceVoxelsFolderName);
    std::string out = vg->spaceCamsTracksDir;
    delete vg;
    return out;
}

largeScale* largeScale::cloneSpaceIfDoesNotExists(int newOcTreeDim, std::string newSpaceFolderName)
{
    if(isSpaceSaved())
    {
        loadSpaceFromFile();
        
        if(doVisualizeVoxels)
            visualizeVoxels();

        largeScale* out = new largeScale(mp, pc, newSpaceFolderName);
        if(out->isSpaceSaved())
        {
            out->loadSpaceFromFile();
            if(doVisualizeVoxels)
                out->visualizeVoxels();
            return out;
        }

        out->space = space;
        out->dimensions = dimensions;
        out->doVisualize = doVisualize;

        out->maxOcTreeDim = (int)((float)maxOcTreeDim / (1024.0f / (float)newOcTreeDim));

        if(mp->verbose)
            printf("maxOcTreeDim new %i\n", out->maxOcTreeDim);
        if(mp->verbose)
            printf("maxOcTreeDim old %i\n", maxOcTreeDim);

        long t1 = clock();

        voxelsGrid* vgactual = new voxelsGrid(dimensions, &space[0], mp, pc, spaceVoxelsFolderName, doVisualize);
        if(maxOcTreeDim == out->maxOcTreeDim)
        {
            voxelsGrid* vgnew = vgactual->copySpace(out->spaceVoxelsFolderName);
            vgnew->generateCamsPtsFromVoxelsTracks();
            delete vgnew;
        }
        else
        {
            voxelsGrid* vgnew = vgactual->cloneSpace(out->maxOcTreeDim, out->spaceVoxelsFolderName);
            vgnew->generateCamsPtsFromVoxelsTracks();
            delete vgnew;
        }
        delete vgactual;

        out->saveSpaceToFile();
        if(doVisualizeVoxels)
            out->visualizeVoxels();

        if(mp->verbose)
            printfElapsedTime(t1, "space cloned in:");

        return out;
    }

    return nullptr;
}

bool largeScale::generateSpace(int maxPts, int ocTreeDim)
{
    if(isSpaceSaved())
    {
        loadSpaceFromFile();
        return false;
    }

    maxOcTreeDim = 1024;
    initialEstimateSpace(maxOcTreeDim);
    maxOcTreeDim = ocTreeDim;

    bool addRandomNoise = mp->mip->_ini.get<bool>("largeScale.addRandomNoise", false);
    float addRandomNoisePercNoisePts =
        (float)mp->mip->_ini.get<double>("largeScale.addRandomNoisePercNoisePts", 10.0);
    int addRandomNoiseNoisPixSizeDistHalfThr =
        (float)mp->mip->_ini.get<int>("largeScale.addRandomNoiseNoisPixSizeDistHalfThr", 10);

    std::string depthMapsPtsSimsTmpDir = generateTempPtsSimsFiles(
        spaceFolderName, mp, addRandomNoise, addRandomNoisePercNoisePts, addRandomNoiseNoisPixSizeDistHalfThr);

    printf("CREATING TRACKS %i %i %i\n", dimensions.x, dimensions.y, dimensions.z);
    staticVector<point3d>* reconstructionPlan = new staticVector<point3d>(1000000);

    std::string tmpdir = spaceFolderName + "tmp/";
    bfs::create_directory(tmpdir);
    voxelsGrid* vg = new voxelsGrid(dimensions, &space[0], mp, pc, tmpdir, doVisualize);
    int maxlevel = 0;
    vg->generateTracksForEachVoxel(reconstructionPlan, maxOcTreeDim, maxPts, 1, maxlevel, depthMapsPtsSimsTmpDir);
    if(mp->verbose)
        printf("max rec level is %i\n", maxlevel);
    for(int i = 1; i < maxlevel; i++)
    {
        dimensions = dimensions * 2;
        maxOcTreeDim = maxOcTreeDim / 2;
        if(mp->verbose)
            printf("dimmension is %i,%i,%i  %i\n", dimensions.x, dimensions.y, dimensions.z, maxOcTreeDim);
    }
    if(mp->verbose)
        printf("final dimmension is %i,%i,%i  %i\n", dimensions.x, dimensions.y, dimensions.z, maxOcTreeDim);

    voxelsGrid* vgnew = new voxelsGrid(dimensions, &space[0], mp, pc, spaceVoxelsFolderName, doVisualize);
    vg->generateSpace(vgnew, voxel(0, 0, 0), dimensions, depthMapsPtsSimsTmpDir);
    vgnew->generateCamsPtsFromVoxelsTracks();
    if(doVisualize)
        vgnew->vizualize();

    delete vgnew;
    delete vg;

    DeleteDirectory(tmpdir);

    deleteTempPtsSimsFiles(mp, depthMapsPtsSimsTmpDir);

    saveArrayToFile<point3d>(spaceFolderName + "spacePatitioning.bin", reconstructionPlan);
    delete reconstructionPlan;

    saveSpaceToFile();

    if(doVisualizeVoxels)
        visualizeVoxels();

    return true;
}

point3d largeScale::getSpaceSteps()
{
    point3d vx = space[1] - space[0];
    point3d vy = space[3] - space[0];
    point3d vz = space[4] - space[0];
    point3d sv;
    sv.x = (vx.size() / (float)dimensions.x) / (float)maxOcTreeDim;
    sv.y = (vy.size() / (float)dimensions.y) / (float)maxOcTreeDim;
    sv.z = (vz.size() / (float)dimensions.z) / (float)maxOcTreeDim;
    return sv;
}
