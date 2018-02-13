// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "fileIO.hpp"
#include <aliceVision/common/common.hpp>
#include <aliceVision/common/MultiViewParams.hpp>
#include <aliceVision/imageIO/image.hpp>

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

bool FileExists(const std::string& filePath)
{
    return boost::filesystem::exists(filePath);
}

bool FolderExists(const std::string& folderPath)
{
    return boost::filesystem::is_directory(folderPath);
}

std::string mv_getFileNamePrefixRcTc(const std::string& baseDir, multiviewInputParams* mip, int rc, int tc)
{
    return baseDir + mip->prefix + num2strFourDecimal(rc) + "_" + num2strFourDecimal(tc);
}

std::string mv_getFileNamePrefix(const std::string& baseDir, multiviewInputParams* mip, int index)
{
    return baseDir + mip->prefix + num2strFourDecimal(index);
}

std::string mv_getFileName(multiviewInputParams* mip, int index, EFileType mv_file_type, int scale)
{
    std::string suffix;
    std::string ext;
    std::string baseDir = mip->mvDir;

    switch(mv_file_type)
    {
        case EFileType::P:
        {
            suffix = "_P";
            ext = "txt";
            break;
        }
        case EFileType::K:
        {
            suffix = "_K";
            ext = "txt";
            break;
        }
        case EFileType::iK:
        {
            suffix = "_iK";
            ext = "txt";
            break;
        }
        case EFileType::R:
        {
            suffix = "_R";
            ext = "txt";
            break;
        }
        case EFileType::iR:
        {
            suffix = "_iR";
            ext = "txt";
            break;
        }
        case EFileType::C:
        {
            suffix = "_C";
            ext = "txt";
            break;
        }
        case EFileType::iP:
        {
            suffix = "_iP";
            ext = "txt";
            break;
        }
        case EFileType::har:
        {
            suffix = "_har";
            ext = "bin";
            break;
        }
        case EFileType::prematched:
        {
            suffix = "_prematched";
            ext = "bin";
            break;
        }
        case EFileType::seeds:
        {
            suffix = "_seeds";
            ext = "bin";
            break;
        }
        case EFileType::growed:
        {
            suffix = "_growed";
            ext = "bin";
            break;
        }
        case EFileType::occMap:
        {
            suffix = "_occMap";
            ext = "bin";
            break;
        }
        case EFileType::nearMap:
        {
            suffix = "_nearMap";
            ext = "bin";
            break;
        }
        case EFileType::op:
        {
            suffix = "_op";
            ext = "bin";
            break;
        }
        case EFileType::wshed:
        {
            suffix = "_wshed";
            ext = "bin";
            break;
        }
        case EFileType::seeds_prm:
        {
            suffix = "_seeds_prm";
            ext = "bin";
            break;
        }
        case EFileType::seeds_flt:
        {
            suffix = "_seeds_flt";
            ext = "bin";
            break;
        }
        case EFileType::img:
        {
            suffix = "_img";
            ext = "bin";
            break;
        }
        case EFileType::imgT:
        {
            suffix = "_imgT";
            ext = "bin";
            break;
        }
        case EFileType::seeds_seg:
        {
            suffix = "_seeds_seg";
            ext = "bin";
            break;
        }
        case EFileType::graphCutMap:
        {
            suffix = "_graphCutMap";
            ext = "bin";
            break;
        }
        case EFileType::graphCutPts:
        {
            suffix = "_graphCutPts";
            ext = "bin";
            break;
        }
        case EFileType::growedMap:
        {
            suffix = "_growedMap";
            ext = "bin";
            break;
        }
        case EFileType::agreedMap:
        {
            suffix = "_agreedMap";
            ext = "bin";
            break;
        }
        case EFileType::agreedPts:
        {
            suffix = "_agreedPts";
            ext = "bin";
            break;
        }
        case EFileType::refinedMap:
        {
            suffix = "_refinedMap";
            ext = "bin";
            break;
        }
        case EFileType::seeds_sfm:
        {
            suffix = "_seeds_sfm";
            ext = "bin";
            break;
        }
        case EFileType::radial_disortion:
        {
            suffix = "_rd";
            ext = "bin";
            break;
        }
        case EFileType::graphCutMesh:
        {
            suffix = "_graphCutMesh";
            ext = "bin";
            break;
        }
        case EFileType::agreedMesh:
        {
            suffix = "_agreedMesh";
            ext = "bin";
            break;
        }
        case EFileType::nearestAgreedMap:
        {
            suffix = "_nearestAgreedMap";
            ext = "bin";
            break;
        }
        case EFileType::segPlanes:
        {
            suffix = "_segPlanes";
            ext = "bin";
            break;
        }
        case EFileType::agreedVisMap:
        {
            suffix = "_agreedVisMap";
            ext = "bin";
            break;
        }
        case EFileType::diskSizeMap:
        {
            suffix = "_diskSizeMap";
            ext = "bin";
            break;
        }
        case EFileType::depthMap:
        {
            if(scale == 0)
                baseDir = mip->_depthMapFilterFolder;
            else
                baseDir = mip->_depthMapFolder;
            suffix = "_depthMap";
            ext = "exr";
            break;
        }
        case EFileType::simMap:
        {
            if(scale == 0)
                baseDir = mip->_depthMapFilterFolder;
            else
                baseDir = mip->_depthMapFolder;
            suffix = "_simMap";
            ext = "exr";
            break;
        }
        case EFileType::mapPtsTmp:
        {
            suffix = "_mapPts";
            ext = "tmp";
            break;
        }
        case EFileType::mapPtsSimsTmp:
        {
            suffix = "_mapPtsSims";
            ext = "tmp";
            break;
        }
        case EFileType::depthMapInfo:
        {
            baseDir = mip->_depthMapFolder;
            suffix = "_depthMapInfo";
            ext = "tmp";
            break;
        }
        case EFileType::camMap:
        {
            suffix = "_camMap";
            ext = "bin";
            break;
        }
        case EFileType::nmodMap:
        {
            baseDir = mip->_depthMapFilterFolder;
            suffix = "_nmodMap";
            ext = "png";
            break;
        }
        case EFileType::D:
        {
            suffix = "_D";
            ext = "txt";
            break;
        }
    }
    if(scale > 1)
    {
        suffix += "_scale" + num2str(scale);
    }

    std::string fileName = mv_getFileNamePrefix(baseDir, mip, index) + suffix + "." + ext;
    return fileName;
}

FILE* mv_openFile(multiviewInputParams* mip, int index, EFileType mv_file_type, const char* readWrite)
{
    const std::string fileName = mv_getFileName(mip, index, mv_file_type);
    FILE* out = fopen(fileName.c_str(), readWrite);
    if (out==NULL)
        throw std::runtime_error(std::string("Cannot create file: ") + fileName);
    return out;
}

Point3d load3x1MatrixFromFile(FILE* fi)
{
    Point3d m;

    float a, b, c;
    fscanf(fi, "%f \n", &a);
    fscanf(fi, "%f \n", &b);
    fscanf(fi, "%f \n", &c);

    m.x = a;
    m.y = b;
    m.z = c;

    return m;
}

Matrix3x3 load3x3MatrixFromFile(FILE* fi)
{
    Matrix3x3 m;

    // M[col*3 + row]
    float a, b, c;
    fscanf(fi, "%f %f %f \n", &a, &b, &c);
    m.m11 = a;
    m.m12 = b;
    m.m13 = c;
    fscanf(fi, "%f %f %f \n", &a, &b, &c);
    m.m21 = a;
    m.m22 = b;
    m.m23 = c;
    fscanf(fi, "%f %f %f \n", &a, &b, &c);
    m.m31 = a;
    m.m32 = b;
    m.m33 = c;

    return m;
}

Matrix3x4 load3x4MatrixFromFile(FILE* fi)
{
    Matrix3x4 m;

    // M[col*3 + row]
    float a, b, c, d;
    fscanf(fi, "%f %f %f %f \n", &a, &b, &c, &d);
    m.m11 = a;
    m.m12 = b;
    m.m13 = c;
    m.m14 = d;
    fscanf(fi, "%f %f %f %f \n", &a, &b, &c, &d);
    m.m21 = a;
    m.m22 = b;
    m.m23 = c;
    m.m24 = d;
    fscanf(fi, "%f %f %f %f \n", &a, &b, &c, &d);
    m.m31 = a;
    m.m32 = b;
    m.m33 = c;
    m.m34 = d;

    return m;
}

Point3d loadPoint3dFromFile(FILE* fi)
{
    Point3d m;

    // M[col*3 + row]
    float a, b, c;
    fscanf(fi, "%f %f %f %*f \n", &a, &b, &c);
    m.x = a;
    m.y = b;
    m.z = c;

    return m;
}

int get2dPointsNum(int imgFileId, multiviewInputParams* mip)
{
    FILE* f = mv_openFile(mip, imgFileId, EFileType::har, "r");

    if(f == nullptr)
    {
        return 0;
    }

    int n = 0;
    while(feof(f) == 0)
    {
        float a, b;
        fscanf(f, "%f %f \n", &a, &b);
        n++;
    }

    fclose(f);

    return n;
}

int load2dPoints(Point2d** _out, int imgFileId, multiviewInputParams* mip)
{
    Point2d* out;
    int n = get2dPointsNum(imgFileId, mip);
    if(n == 0)
    {
        out = new Point2d[1];
    }
    else
    {
        out = new Point2d[n];

        FILE* f = mv_openFile(mip, imgFileId, EFileType::har, "r");

        int i = 0;
        while(feof(f) == 0)
        {
            float a, b;
            fscanf(f, "%f %f \n", &a, &b);
            out[i] = Point2d((float)a, (float)b);
            i++;
        }

        fclose(f);
    }

    *_out = out;

    return n;
}

StaticVector<Point2d>* load2dPoints(int imgFileId, multiviewInputParams* mip)
{
    int n = get2dPointsNum(imgFileId, mip);

    StaticVector<Point2d>* out = new StaticVector<Point2d>(std::max(n, 1));

    if(n > 0)
    {
        FILE* f = mv_openFile(mip, imgFileId, EFileType::har, "r");
        while(feof(f) == 0)
        {
            float a, b;
            fscanf(f, "%f %f \n", &a, &b);
            out->push_back(Point2d((float)a, (float)b));
        }
        fclose(f);
    }

    return out;
}

void load2dPoints(StaticVector<Point2d>* out, int imgFileId, multiviewInputParams* mip)
{
    out->resize(0);
    FILE* f = mv_openFile(mip, imgFileId, EFileType::har, "r");
    while(feof(f) == 0)
    {
        float a, b;
        fscanf(f, "%f %f \n", &a, &b);
        out->push_back(Point2d((float)a, (float)b));
    }
    fclose(f);
}

void memcpyRGBImageFromFileToArr(int camId, Color* imgArr, const std::string& fileNameOrigStr, multiviewInputParams* mip,
                                 bool transpose, int scaleFactor, int bandType)
{
    int w = mip->getWidth(camId) / std::max(scaleFactor, 1);
    int h = mip->getHeight(camId) / std::max(scaleFactor, 1);

    int origWidth, origHeight;
    std::vector<Color> cimg;
    imageIO::readImage(fileNameOrigStr, origWidth, origHeight, cimg);

    // check image size...
    if((mip->getWidth(camId) != origWidth) || (mip->getHeight(camId) != origHeight))
    {
        std::stringstream s;
        s << "Bad image dimension for camera : " << camId << "\n";
        s << "- image path : " << fileNameOrigStr << "\n";
        s << "- expected dimension : " << mip->getWidth(camId) << "x" << mip->getHeight(camId) << "\n";
        s << "- real dimension : " << origWidth << "x" << origHeight << "\n";
        throw std::runtime_error(s.str());
    }

    if(scaleFactor > 1)
    {
        std::vector<Color> bmpr;
        imageIO::resizeImage(origWidth, origHeight, scaleFactor, cimg, bmpr);
        cimg = bmpr;
    }

    if(bandType == 1)
    {

        // IplImage* cimg1=cvCreateImage(cvSize(cimg->width,cimg->height),IPL_DEPTH_8U,3);
        // cvSmooth(cimg1,cimg,CV_BILATERAL,11,0,10.0,5.0);
        // cvReleaseImage(&cimg);
        // cimg=cimg1;
        std::vector<Color> smooth;
        imageIO::convolveImage(w, h, cimg, smooth, "gaussian", 11.0f, 11.0f);
        cimg = smooth;
    }

    if(bandType == 2)
    {
        std::vector<Color> bmps;
        imageIO::convolveImage(w, h, cimg, bmps, "gaussian", 11.0f, 11.0f);

        for(int y = 0; y < h; y++)
        {
            for(int x = 0; x < w; x++)
            {
                const std::size_t index = y * w + x;
                Color& cimc = cimg.at(index);
                cimc = cimc - bmps.at(index); //cimg(x, y) - bmps(x, y)
            }
        }
    }

    for(int y = 0; y < h; y++)
    {
        for(int x = 0; x < w; x++)
        {
            const Color color = cimg.at(y * w + x);

            if(transpose)
                imgArr[y * w + x] = color;
            else
                imgArr[x * h + y] = color;

        }
    }
}


void saveSeedsToFile(StaticVector<SeedPoint>* seeds, const std::string& fileName)
{
    FILE* f = fopen(fileName.c_str(), "wb");
    if(f == nullptr)
    {
        //
    }
    else
    {
        int size = seeds->size();
        fwrite(&size, sizeof(int), 1, f);
        for(int i = 0; i < size; i++)
        {
            SeedPoint* sp = &(*seeds)[i];
            seed_io_block sb;
            sb.area = sp->area;
            sb.ncams = sp->cams.size();
            sb.op = sp->op;
            sb.pixSize = sp->pixSize;
            sb.segId = sp->segId;
            sb.xax = sp->xax;
            sb.yax = sp->yax;

            fwrite(&sb, sizeof(seed_io_block), 1, f);

            for(int j = 0; j < sb.ncams; j++)
            {
                unsigned short c = sp->cams[j];
                fwrite(&c, sizeof(unsigned short), 1, f);
            }

            for(int j = 0; j < sb.ncams + 1; j++)
            {
                Point2d sh = sp->cams.shifts[j];
                fwrite(&sh, sizeof(Point2d), 1, f);
            }
        }
        fclose(f);
    }
}

void saveSeedsToFile(StaticVector<SeedPoint>* seeds, int refImgFileId, multiviewInputParams* mip, EFileType mv_file_type)
{
    std::string fileName = mv_getFileName(mip, refImgFileId, mv_file_type);
    saveSeedsToFile(seeds, fileName);
}

bool loadSeedsFromFile(StaticVector<SeedPoint>** seeds, const std::string& fileName)
{
    FILE* f = fopen(fileName.c_str(), "rb");
    if(f == nullptr)
    {
        // printf("file does not exists \n");
        *seeds = new StaticVector<SeedPoint>(1);
        return false;
    }

    int size;
    fread(&size, sizeof(int), 1, f);
    *seeds = new StaticVector<SeedPoint>(std::max(1, size));

    for(int i = 0; i < size; i++)
    {
        SeedPoint sp = SeedPoint();
        seed_io_block sb;

        fread(&sb, sizeof(seed_io_block), 1, f);

        sp.area = sb.area;
        sp.cams.reserve(sb.ncams);
        sp.op = sb.op;

        // printf("seed rx: %i, ry: %i \n", (int)sp->op.n.rx, (int)sp->op.n.ry);

        sp.pixSize = sb.pixSize;
        sp.segId = sb.segId;
        sp.xax = sb.xax;
        sp.yax = sb.yax;

        for(int j = 0; j < sb.ncams; j++)
        {
            unsigned short c;
            fread(&c, sizeof(unsigned short), 1, f);
            sp.cams.push_back(c);
        }

        for(int j = 0; j < sb.ncams + 1; j++)
        {
            Point2d sh;
            fread(&sh, sizeof(Point2d), 1, f);
            sp.cams.shifts[j] = sh;
        }

        (*seeds)->push_back(sp);
    }

    fclose(f);
    return true;
}

bool loadSeedsFromFile(StaticVector<SeedPoint>** seeds, int refImgFileId, multiviewInputParams* mip, EFileType mv_file_type)
{
    std::string fileName = mv_getFileName(mip, refImgFileId, mv_file_type);
    return loadSeedsFromFile(seeds, fileName);
}

int getSeedsSizeFromFile(int refImgFileId, multiviewInputParams* mip, EFileType mv_file_type)
{
    FILE* f = mv_openFile(mip, refImgFileId, mv_file_type, "rb");
    if(f == nullptr)
    {
        return 0;
    }

    int size;
    fread(&size, sizeof(int), 1, f);
    fclose(f);
    return size;
}

int getGrowedSizeFromFile(int refImgFileId, multiviewInputParams* mip)
{
    FILE* f = mv_openFile(mip, refImgFileId, EFileType::growed, "rb");
    if(f == nullptr)
    {
        //
    }
    else
    {
        int size;
        fread(&size, sizeof(int), 1, f);
        fclose(f);
        return size;
    }

    return 0;
}

void saveUniqueIdAliveToFile(std::vector<bool>* uniqueIdAlive, multiviewInputParams* mip)
{
    std::string fname = mip->mvDir + mip->prefix + "_alive.txt";
    FILE* f = fopen(fname.c_str(), "wb");
    if(f == nullptr)
    {
        //
    }
    else
    {
        int32_t sizei = (int32_t)uniqueIdAlive->size();
        fwrite(&sizei, sizeof(int32_t), 1, f);

        for(int i = 0; i < sizei; i++)
        {
            bool b = (*uniqueIdAlive)[i];
            fwrite(&b, sizeof(bool), 1, f);
        }
        fclose(f);
    }
}

void loadUniqueIdAliveFromFile(std::vector<bool>* uniqueIdAlive, multiviewInputParams* mip)
{
    std::string fname = mip->mvDir + mip->prefix + "_alive.txt";
    FILE* f = fopen(fname.c_str(), "rb");
    if(f == nullptr)
    {
        //
    }
    else
    {
        int32_t size;
        fread(&size, sizeof(int32_t), 1, f);
        uniqueIdAlive->reserve(size);
        for(int i = 0; i < size; i++)
        {
            bool b;
            fread(&b, sizeof(bool), 1, f);
            uniqueIdAlive->push_back(b);
        }
        fclose(f);
    }
}

void deleteFilesOfType(multiviewInputParams& mip, int ncams, EFileType mv_file_type)
{
    // delete files
    long t1 = initEstimate();
    for(int rc = 0; rc < ncams; rc++)
    {
        std::string fileName = mv_getFileName(&mip, rc + 1, mv_file_type);
        remove(fileName.c_str());
        printfEstimate(rc, ncams, t1);
    }
    finishEstimate();
}

void saveOrientedPointsToFile(StaticVector<OrientedPoint>* ops, int refImgFileId, multiviewInputParams* mip)
{
    FILE* f = mv_openFile(mip, refImgFileId, EFileType::op, "wb");
    if(f == nullptr)
    {
        //
    }
    else
    {
        int size = ops->size();
        fwrite(&size, sizeof(int), 1, f);
        fwrite(&(*ops)[0], sizeof(OrientedPoint), size, f);
        fclose(f);
    }
}

StaticVector<OrientedPoint>* loadOrientedPointsFromFile(int refImgFileId, multiviewInputParams* mip)
{
    FILE* f = mv_openFile(mip, refImgFileId, EFileType::op, "rb");
    if(f == nullptr)
    {
        //
    }
    else
    {
        int size;
        fread(&size, sizeof(int), 1, f);
        StaticVector<OrientedPoint>* ops = new StaticVector<OrientedPoint>(size);
        ops->resize(size);
        fread(&(*ops)[0], sizeof(OrientedPoint), size, f);
        fclose(f);

        return ops;
    }

    return nullptr;
}

int getNumOrientedPointsFromFile(int refImgFileId, multiviewInputParams* mip)
{
    FILE* f = mv_openFile(mip, refImgFileId, EFileType::op, "rb");
    if(f == nullptr)
    {
        //
    }
    else
    {
        int size;
        fread(&size, sizeof(int), 1, f);
        fclose(f);

        return size;
    }

    return 0;
}

/*
void loadVisibilityMapFromFileWithAllocation(bool **vis, int refImgFileId, multiviewInputParams *mip)
{
        FILE *f = mv_openFile(mip, refImgFileId, EFileType::visibility_map, "rb");
        if (f == NULL)
        {
                //
        }else
        {
                int size;
                fread(&size, sizeof(int), 1, f);
                *vis = new bool[size];
                fread(&((*vis)[0]), sizeof(bool), size, f);
                fclose(f);
        };
}

void saveVisibilityMapToFile(bool *vis, int size, int refImgFileId, multiviewInputParams *mip)
{
        FILE *f = mv_openFile(mip, refImgFileId, EFileType::visibility_map, "wb");
        if (f == NULL)
        {
                //
        }else
        {
                fwrite(&size, sizeof(int), 1, f);
                fwrite(&vis[0], sizeof(bool), size, f);
                fclose(f);
        };
}
*/

bool loadPairConfidenceMatrixFromFileWithAllocation(unsigned char** cm, multiviewInputParams* mip,
                                                    const std::string& name)
{
    std::string fname = mip->mvDir + mip->prefix + name;
    FILE* f = fopen(fname.c_str(), "rb");
    if(f == nullptr)
    {
        *cm = nullptr;
        return false;
    }

    int32_t size;
    fread(&size, sizeof(int32_t), 1, f);
    *cm = new unsigned char[size];
    fread(&((*cm)[0]), sizeof(unsigned char), size, f);
    fclose(f);
    return true;
}

void savePairConfidenceMatrixToFile(unsigned char* cm, int32_t size, multiviewInputParams* mip, const std::string& name)
{
    std::string fname = mip->mvDir + mip->prefix + name;
    FILE* f = fopen(fname.c_str(), "wb");
    if(f == nullptr)
    {
        //
    }
    else
    {
        fwrite(&size, sizeof(int32_t), 1, f);
        fwrite(&cm[0], sizeof(unsigned char), size, f);
        fclose(f);
    }
}

void deleteAllFiles(multiviewInputParams* mip)
{
    printf("deleteing temporary files\n");
    // deletePremtachFiles(*mip,mip->getNbCameras());
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::seeds);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::growed);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::nearMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::occMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::op);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::seeds_flt);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::seeds_prm);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::seeds_sfm);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::refinedMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::growedMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::graphCutPts);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::graphCutMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::graphCutMesh);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::agreedMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::agreedPts);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::agreedMesh);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::nearMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::occMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::op);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::nearestAgreedMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::segPlanes);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::agreedVisMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::diskSizeMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::depthMap);
    deleteFilesOfType(*mip, mip->getNbCameras(), EFileType::simMap);
}

bool getDepthMapInfo(int refImgFileId, multiviewInputParams* mip, float& mindepth, float& maxdepth,
                     StaticVector<int>** tcams)
{
    *tcams = nullptr;
    FILE* f = mv_openFile(mip, refImgFileId, EFileType::depthMapInfo, "r");
    if(f == nullptr)
    {
        printf("WARNING!!!! depth map info %i does not exists!\n", refImgFileId);
        return false;
    }

    int ntcams;
    fscanf(f, "minDepth %f, maxDepth %f, ntcams %i, tcams", &mindepth, &maxdepth, &ntcams);
    (*tcams) = new StaticVector<int>(ntcams);
    for(int c = 0; c < ntcams; c++)
    {
        int tc;
        fscanf(f, " %i", &tc);
        (*tcams)->push_back(tc);
    }
    fclose(f);
    return true;
}

bool getDepthMapInfoDepthLimits(int refImgFileId, multiviewInputParams* mip, float& mindepth, float& maxdepth)
{
    FILE* f = mv_openFile(mip, refImgFileId, EFileType::depthMapInfo, "r");
    if(f == nullptr)
    {
        printf("WARNING!!!!\n");
        return false;
    }

    int ntcams;
    fscanf(f, "minDepth %f, maxDepth %f, ntcams %i, tcams", &mindepth, &maxdepth, &ntcams);
    fclose(f);
    return true;
}

bool IsDots(const char* str)
{
    std::string dd = ".";
    std::string ddd = ".";
    return !((strcmp(str, dd.c_str()) != 0) && (strcmp(str, ddd.c_str()) != 0));
}

bool DeleteDirectory(const std::string& sPath)
{
    boost::filesystem::remove_all(sPath);
    return true;
}

bool getDirectoryFiles(std::vector<std::string>& out, const std::string& sPath, const std::string& ext)
{
    namespace fs = boost::filesystem;

    fs::path apk_path(sPath);
    fs::directory_iterator end;

    for(fs::directory_iterator i(apk_path); i != end; ++i)
    {
        if(ext.empty() || i->path().extension() == ext)
            out.push_back(i->path().string());
    }

    return true;
}

void readSifts(const std::string& fileName, StaticVector<float>** descriptors, StaticVector<SiftKeypoint>** keys)
{
    FILE* f = fopen(fileName.c_str(), "rb");

    int n;
    fread(&n, sizeof(int), 1, f);

    // printf("%i\n",n);
    // float mbsifts = ((float)(n*sizeof(SiftGPU::SiftKeypoint)+n*128*sizeof(float))/1024.0)/1024.0;
    // printf("%i sifts contains %f MB \n",n,mbsifts);

    StaticVector<SiftKeypoint>* Keys = new StaticVector<SiftKeypoint>(n);
    Keys->resize(n);
    StaticVector<float>* Descriptors = new StaticVector<float>(128 * n);
    Descriptors->resize(128 * n);

    fread(&(*Keys)[0], sizeof(SiftKeypoint), n, f);
    fread(&(*Descriptors)[0], sizeof(float), 128 * n, f);

    fclose(f);

    (*keys) = Keys;
    (*descriptors) = Descriptors;
}

void splitString(const std::string& str, const std::string& delimiters, std::vector<std::string>& tokens)
{
    // Skip delimiters at beginning.
    std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
    // Find first "non-delimiter".
    std::string::size_type pos = str.find_first_of(delimiters, lastPos);

    while(std::string::npos != pos || std::string::npos != lastPos)
    {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(delimiters, pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(delimiters, lastPos);
    }
}
