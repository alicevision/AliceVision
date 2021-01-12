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
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>
#include <aliceVision/mvsUtils/ImagesCache.hpp>
#include <aliceVision/mvsData/imageIO.hpp>

#include <fstream>
#include <string>

#include <OpenImageIO/paramlist.h>

namespace oiio = OIIO;

namespace aliceVision {
namespace mvsUtils {

bool FileExists(const std::string& filePath);
bool FolderExists(const std::string& folderPath);

std::string getFileNameFromViewId(const MultiViewParams& mp, int viewId, EFileType fileType, int scale = 0, const std::string& customSuffix = "");

std::string getFileNameFromIndex(const MultiViewParams& mp, int index, EFileType fileType, int scale = 0, const std::string& customSuffix = "");

FILE* mv_openFile(const MultiViewParams& mp, int index, EFileType mv_file_type, const char* readWrite);
Matrix3x4 load3x4MatrixFromFile(FILE* fi);


template<class Image>
void loadImage(const std::string& path, const MultiViewParams& mp, int camId, Image& img,
    imageIO::EImageColorSpace colorspace, ECorrectEV correctEV);

bool DeleteDirectory(const std::string& sPath);

} // namespace mvsUtils
} // namespace aliceVision
