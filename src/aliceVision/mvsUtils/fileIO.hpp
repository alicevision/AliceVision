// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Color.hpp>
#include <aliceVision/mvsData/Matrix3x4.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/mvsData/SeedPoint.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>

#include <fstream>

namespace aliceVision {
namespace mvsUtils {

bool FileExists(const std::string& filePath);
bool FolderExists(const std::string& folderPath);

std::string mv_getFileNamePrefix(const std::string& baseDir, const MultiViewParams* mp, int index);
std::string mv_getFileName(const MultiViewParams* mp, int index, EFileType mv_file_type, int scale = 0);
FILE* mv_openFile(const MultiViewParams* mp, int index, EFileType mv_file_type, const char* readWrite);
Matrix3x4 load3x4MatrixFromFile(FILE* fi);
void memcpyRGBImageFromFileToArr(int camId, Color* imgArr, const std::string& fileNameOrigStr, const MultiViewParams* mp,
                                 int bandType);
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
void saveSeedsToFile(StaticVector<SeedPoint>* seeds, int refImgFileId, const MultiViewParams* mp, EFileType mv_file_type);
bool loadSeedsFromFile(StaticVector<SeedPoint>** seeds, const std::string& fileName);
bool loadSeedsFromFile(StaticVector<SeedPoint>** seeds, int refImgFileId, const MultiViewParams* mp, EFileType mv_file_type);
bool getDepthMapInfo(int refImgFileId, const MultiViewParams* mp, float& mindepth, float& maxdepth,
                     StaticVector<int>** tcams);
bool DeleteDirectory(const std::string& sPath);

} // namespace mvsUtils
} // namespace aliceVision
