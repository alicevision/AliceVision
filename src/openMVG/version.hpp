// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef ALICEVISION_VERSION_H_
#define ALICEVISION_VERSION_H_

#define ALICEVISION_VERSION_MAJOR 0
#define ALICEVISION_VERSION_MINOR 9
#define ALICEVISION_VERSION_REVISION 5

// Preprocessor to string conversion
#define ALICEVISION_TO_STRING_HELPER(x) #x
#define ALICEVISION_TO_STRING(x) ALICEVISION_TO_STRING_HELPER(x)

// AliceVision version as a string; for example "0.9.0".
#define ALICEVISION_VERSION_STRING ALICEVISION_TO_STRING(ALICEVISION_VERSION_MAJOR) "." \
                             ALICEVISION_TO_STRING(ALICEVISION_VERSION_MINOR) "." \
                             ALICEVISION_TO_STRING(ALICEVISION_VERSION_REVISION)

#endif  // ALICEVISION_VERSION_H_
