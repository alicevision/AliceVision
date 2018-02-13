// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/Color.hpp>
#include <aliceVision/structures/Matrix3x3.hpp>
#include <aliceVision/structures/Point2d.hpp>
#include <aliceVision/structures/Point3d.hpp>
#include <aliceVision/structures/SeedPoint.hpp>
#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/common/MultiViewParams.hpp>

#include <fstream>

struct multiviewInputParams;

bool FileExists(const std::string& filePath);
bool FolderExists(const std::string& folderPath);

std::string mv_getFileNamePrefixRcTc(const std::string& baseDir, multiviewInputParams* mip, int rc, int tc);
std::string mv_getFileNamePrefix(const std::string& baseDir, multiviewInputParams* mip, int index);
std::string mv_getFileName(multiviewInputParams* mip, int index, EFileType mv_file_type, int scale = 0);
FILE* mv_openFile(multiviewInputParams* mip, int index, EFileType mv_file_type, const char* readWrite);
Point3d load3x1MatrixFromFile(FILE* fi);
Matrix3x3 load3x3MatrixFromFile(FILE* fi);
Matrix3x4 load3x4MatrixFromFile(FILE* fi);
Point3d loadPoint3dFromFile(FILE* fi);
int get2dPointsNum(int imgFileId, multiviewInputParams* mip);
int load2dPoints(Point2d** _out, int imgFileId, multiviewInputParams* mip);
StaticVector<Point2d>* load2dPoints(int imgFileId, multiviewInputParams* mip);
void load2dPoints(StaticVector<Point2d>* out, int imgFileId, multiviewInputParams* mip);
void memcpyRGBImageFromFileToArr(int camId, Color* imgArr, const std::string& fileNameOrigStr, multiviewInputParams* mip,
                                 bool transpose, int scaleFactor, int bandType);
struct seed_io_block            // 80 bytes
{
    OrientedPoint op;           // 28 bytes
    Point3d xax;                // 12 bytes
    Point3d yax;                // 12 bytes
    float pixSize;              // 4 bytes
    uint64_t area;              // 8 bytes
    uint64_t segId;             // 8 bytes
    unsigned short ncams;       // 2 bytes
    unsigned short padding[3];  // 6 bytes
};

void saveSeedsToFile(StaticVector<SeedPoint>* seeds, const std::string& fileName);
void saveSeedsToFile(StaticVector<SeedPoint>* seeds, int refImgFileId, multiviewInputParams* mip, EFileType mv_file_type);
bool loadSeedsFromFile(StaticVector<SeedPoint>** seeds, const std::string& fileName);
bool loadSeedsFromFile(StaticVector<SeedPoint>** seeds, int refImgFileId, multiviewInputParams* mip, EFileType mv_file_type);
int getSeedsSizeFromFile(int refImgFileId, multiviewInputParams* mip, EFileType mv_file_type);
//int getGrowedSizeFromFile(int refImgFileId, multiviewInputParams* mip);

void saveUniqueIdAliveToFile(std::vector<bool>* uniqueIdAlive, multiviewInputParams* mip);
void loadUniqueIdAliveFromFile(std::vector<bool>* uniqueIdAlive, multiviewInputParams* mip);
void deleteFilesOfType(multiviewInputParams& mip, int ncams, EFileType mv_file_type);
void saveOrientedPointsToFile(StaticVector<OrientedPoint>* ops, int refImgFileId, multiviewInputParams* mip);
StaticVector<OrientedPoint>* loadOrientedPointsFromFile(int refImgFileId, multiviewInputParams* mip);
int getNumOrientedPointsFromFile(int refImgFileId, multiviewInputParams* mip);
bool loadPairConfidenceMatrixFromFileWithAllocation(unsigned char** cm, multiviewInputParams* mip,
                                                    const std::string& name);
void savePairConfidenceMatrixToFile(unsigned char* cm, int32_t size, multiviewInputParams* mip,
                                    const std::string& name);
struct outStruct
{
    OrientedPoint op;
    int tarFeatId;

    outStruct()
    {
        op = OrientedPoint();
        tarFeatId = -1;
    }

    outStruct& operator=(const outStruct param)
    {
        op = param.op;
        tarFeatId = param.tarFeatId;
        return *this;
    }
};

void deleteAllFiles(multiviewInputParams* mip);
bool getDepthMapInfo(int refImgFileId, multiviewInputParams* mip, float& mindepth, float& maxdepth,
                     StaticVector<int>** tcams);
bool getDepthMapInfoDepthLimits(int refImgFileId, multiviewInputParams* mip, float& mindepth, float& maxdepth);
bool DeleteDirectory(const std::string& sPath);
bool getDirectoryFiles(std::vector<std::string>& out, const std::string& sPath, const std::string& ext);

typedef struct SiftKeypoint
{
    float x, y, s, o; // x, y, scale, orientation.
} SiftKeypoint;

void readSifts(const std::string& fileName, StaticVector<float>** descriptors, StaticVector<SiftKeypoint>** keys);

void splitString(const std::string& str, const std::string& delimiters, std::vector<std::string>& tokens);
