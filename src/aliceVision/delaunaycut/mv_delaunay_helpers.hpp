#pragma once
#include <aliceVision/structures/mv_staticVector.hpp>

class multiviewParams;

staticVector<staticVector<int>*>* getPtsCamsFromInfoFile(const std::string& fileNameInfo);
staticVector<int>* getUsedCamsFromInfoFile(const std::string& fileNameInfo, multiviewParams* mp);
