#include "mv_mesh_orthomosaic.hpp"

#include <opencv/cv.h>
#include <opencv/highgui.h>


mv_mesh_orthomosaic::mv_mesh_orthomosaic(std::string _tmpDir, std::string _demName, multiviewParams* _mp)
    : mv_mesh_dem(_tmpDir, -1, -1, _mp)
{
    demName = _demName;

    o3d = new mv_output3D(mp);

    initDEMFromFile(demName);

    demPixS.x = dim.x / (float)demW;
    demPixS.y = dim.y / (float)demH;
    demPixS.z = dim.z / 65535.0f;

    // staticVector<staticVector<int>*> *ptsCams = loadArrayOfArraysFromFile<int>(ptsCamsFileName);
    // camsPts = convertObjectsCamsToCamsObjects(mp, ptsCams);
    // deleteArrayOfArrays<int>(&ptsCams);

    percentile = (float)mp->mip->_ini.get<double>("DEM.percentile", 0.999);

    orm = new staticVector<point4d>(demW * demH);
    orm->resize_with(demW * demH, point4d(0.0f, 0.0f, 0.0f, 0.0f));

    pixSizeMap = new staticVector<float>(demW * demH);
    pixSizeMap->resize_with(demW * demH, 0.0f);

    int bandType = 0;
    ic = new mv_images_cache(mp, bandType, true);

    visualizeMetaData = (bool)mp->mip->_ini.get<bool>("orthomosaic.visualizeMetaData", false);
    doAverage = (bool)mp->mip->_ini.get<bool>("orthomosaic.doAverage", true);
}

mv_mesh_orthomosaic::~mv_mesh_orthomosaic()
{
    delete o3d;
    delete ic;
    // deleteArrayOfArrays<int>(&camsPts);
    delete orm;
    delete pixSizeMap;
}

void mv_mesh_orthomosaic::computeRcOrthomosaic(int rc, staticVector<float>* dem, staticVector<float>* demGlob)
{
    staticVector<point4d>* rcorm = nullptr;

    if(visualizeMetaData)
    {
        rcorm = new staticVector<point4d>(demW * demH);
        rcorm->resize_with(demW * demH, point4d(0.0f, 0.0f, 0.0f, 0.0f));
    }

    for(int y = 0; y < demH; y++)
    {
        for(int x = 0; x < demW; x++)
        {
            float demz = (*dem)[y * demW + x];
            float demzGlob = (*demGlob)[y * demW + x];
            if((demzGlob == demzGlob) && (!std::isnan(demzGlob)) && (demzGlob > 0.0f) && (demzGlob < dim.z) &&
               (demz == demz) && (!std::isnan(demz)) && (demz > 0.0f) && (demz < dim.z))
            {
                point3d p = oax + xax * (demPixS.x * (float)x) + yax * (demPixS.y * (float)y) + zax * demz;
                point2d pix;
                mp->getPixelFor3DPoint(&pix, p, rc);
                if(mp->isPixelInImage(pix))
                {
                    point3d pGlob = oax + xax * (demPixS.x * (float)x) + yax * (demPixS.y * (float)y) + zax * demzGlob;
                    float pixSize = mp->getCamPixelSize(p, rc);
                    float pixSizeThr = 5.0f * pixSize;

                    if((p - pGlob).size() < pixSizeThr)
                    {
                        point3d col = ic->getPixelValueInterpolated(&pix, rc);
                        if(visualizeMetaData)
                        {
                            (*rcorm)[y * demW + x].x += col.x;
                            (*rcorm)[y * demW + x].y += col.y;
                            (*rcorm)[y * demW + x].z += col.z;
                            (*rcorm)[y * demW + x].w += 1.0f;
                        }
                        if(doAverage)
                        {
                            (*orm)[y * demW + x].x += col.x;
                            (*orm)[y * demW + x].y += col.y;
                            (*orm)[y * demW + x].z += col.z;
                            (*orm)[y * demW + x].w += 1.0f;
                            (*pixSizeMap)[y * demW + x] += pixSize;
                        }
                        else
                        {
                            float w = (*orm)[y * demW + x].w;
                            float d = (p - mp->CArr[rc]).size();
                            if((w == 0.0f) || (w > d))
                            {
                                (*orm)[y * demW + x].x = col.x;
                                (*orm)[y * demW + x].y = col.y;
                                (*orm)[y * demW + x].z = col.z;
                                (*orm)[y * demW + x].w = d;
                                (*pixSizeMap)[y * demW + x] = pixSize;
                            }
                        }
                    }
                }
            }
        }
    }

    if(visualizeMetaData)
    {
        for(int i = 0; i < demW * demH; i++)
        {
            if((*rcorm)[i].w > 0.0)
            {
                (*rcorm)[i].x = (*rcorm)[i].x / (*rcorm)[i].w;
                (*rcorm)[i].y = (*rcorm)[i].y / (*rcorm)[i].w;
                (*rcorm)[i].z = (*rcorm)[i].z / (*rcorm)[i].w;
                (*rcorm)[i].w = (*rcorm)[i].w / (*rcorm)[i].w;
            }
        }

        IplImage* img = cvCreateImage(cvSize(demW, demH), IPL_DEPTH_8U, 3);
        IplImage* imgMask = cvCreateImage(cvSize(demW, demH), IPL_DEPTH_8U, 1);
        for(int y = 0; y < demH; y++)
        {
            for(int x = 0; x < demW; x++)
            {
                CvScalar c;
                c.val[2] = (*rcorm)[y * demW + x].x;
                c.val[1] = (*rcorm)[y * demW + x].y;
                c.val[0] = (*rcorm)[y * demW + x].z;
                cvSet2D(img, y, x, c);
                CvScalar cm;
                cm.val[0] = (*rcorm)[y * demW + x].w * 255.0f;
                cvSet2D(imgMask, y, x, cm);
            }
        }

        std::string imageFileName = tmpDir + num2strFourDecimal(rc) + "orthoMosaic.png";
        if(cvSaveImage(imageFileName.c_str(), img) == 0)
            printf("Could not save: %s\n", imageFileName.c_str());

        std::string imageMaskFileName = tmpDir + num2strFourDecimal(rc) + "orthoMosaicMask.png";
        if(cvSaveImage(imageMaskFileName.c_str(), imgMask) == 0)
            printf("Could not save: %s\n", imageMaskFileName.c_str());

        std::string imageRGBAFileName = tmpDir + num2strFourDecimal(rc) + "orthoMosaicMask.tiff";

        char buffer[5000];
        sprintf(buffer, "convert %s %s -compose copy-opacity -composite %s", imageFileName.c_str(),
                imageMaskFileName.c_str(), imageRGBAFileName.c_str());
        system(buffer);

        cvReleaseImage(&img);
        cvReleaseImage(&imgMask);

        delete rcorm;
    }
}

void mv_mesh_orthomosaic::saveDem2Wrl(std::string wrlFileName, staticVector<float>* dem)
{
    int LUX = mp->mip->_ini.get<int>("orthomosaic.LUX", 0);
    int RDX = mp->mip->_ini.get<int>("orthomosaic.RDX", demW);

    int LUY = mp->mip->_ini.get<int>("orthomosaic.LUY", 0);
    int RDY = mp->mip->_ini.get<int>("orthomosaic.RDY", demH);

    mv_mesh* me = new mv_mesh();
    me->pts = new staticVector<point3d>(demW * demH);
    me->tris = new staticVector<mv_mesh::triangle>(2 * demW * demH);

    staticVector<int>* pixId2PtId = new staticVector<int>(demW * demH);
    for(int y = 0; y < demH; y++)
    {
        for(int x = 0; x < demW; x++)
        {
            int ptid = -1;
            if((x >= LUX) && (x <= RDX) && (y >= LUY) && (y <= RDY))
            {
                float demz = (*dem)[y * demW + x];
                if((demz == demz) && (!std::isnan(demz)) && (demz > 0.0f) && (demz < dim.z))
                {
                    point3d p = oax + xax * (demPixS.x * (float)x) + yax * (demPixS.y * (float)y) + zax * demz;
                    me->pts->push_back(p);
                    ptid = me->pts->size() - 1;
                }
            }
            pixId2PtId->push_back(ptid);
        }
    }

    for(int y = LUY; y < RDY - 1; y++)
    {
        for(int x = LUX; x < RDX - 1; x++)
        {
            int id1 = (*pixId2PtId)[y * demW + x];
            int id2 = (*pixId2PtId)[y * demW + (x + 1)];
            int id3 = (*pixId2PtId)[(y + 1) * demW + (x + 1)];
            int id4 = (*pixId2PtId)[(y + 1) * demW + x];

            if((id1 >= 0) && (id2 >= 0) && (id3 >= 0) && (id4 >= 0))
            {
                mv_mesh::triangle t1;
                t1.i[2] = id1;
                t1.i[1] = id2;
                t1.i[0] = id3;
                t1.alive = true;
                me->tris->push_back(t1);
                mv_mesh::triangle t2;
                t2.i[2] = id3;
                t2.i[1] = id4;
                t2.i[0] = id1;
                t2.alive = true;
                me->tris->push_back(t2);
            }
        }
    }

    o3d->saveMvMeshToWrl(me, wrlFileName);

    delete me;
    delete pixId2PtId;
}
