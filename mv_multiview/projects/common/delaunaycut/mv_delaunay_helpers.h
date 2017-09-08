#pragma once
#include "structures/mv_staticVector.h"

class multiviewParams;

staticVector<staticVector<int>*>* getPtsCamsFromInfoFile(const std::string& fileNameInfo);
staticVector<int>* getUsedCamsFromInfoFile(const std::string& fileNameInfo, multiviewParams* mp);
