// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_VERSION_H_
#define OPENMVG_VERSION_H_

#define OPENMVG_VERSION_MAJOR 0
#define OPENMVG_VERSION_MINOR 9
#define OPENMVG_VERSION_REVISION 5

// Preprocessor to string conversion
#define OPENMVG_TO_STRING_HELPER(x) #x
#define OPENMVG_TO_STRING(x) OPENMVG_TO_STRING_HELPER(x)

// AliceVision version as a string; for example "0.9.0".
#define OPENMVG_VERSION_STRING OPENMVG_TO_STRING(OPENMVG_VERSION_MAJOR) "." \
                             OPENMVG_TO_STRING(OPENMVG_VERSION_MINOR) "." \
                             OPENMVG_TO_STRING(OPENMVG_VERSION_REVISION)

#endif  // OPENMVG_VERSION_H_
