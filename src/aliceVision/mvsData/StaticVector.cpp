// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "StaticVector.hpp"

namespace aliceVision {

int getArrayLengthFromFile(std::string fileName)
{
    vfs::istream f{fileName, std::ios_base::binary};
    if (!f)
    {
        // printf("WARNING: file %s does not exists!\n", fileName.c_str());
        return 0;
    }

    int n = 0;
    size_t retval = f.fread(&n, sizeof(int), 1);
    if( retval != sizeof(int) )
    {
        ALICEVISION_LOG_WARNING("[IO] getArrayLengthFromFile: can't read array length (1)");
    }
    if(n == -1)
    {
        retval = f.fread(&n, sizeof(int), 1);
        if( retval != sizeof(int) )
        {
            ALICEVISION_LOG_WARNING("[IO] getArrayLengthFromFile: can't read array length (2)");
        }
    }
    return n;
}

} // namespace aliceVision
