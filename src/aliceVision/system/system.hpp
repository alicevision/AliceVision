// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#if defined(linux) || defined(__linux) || defined(LINUX) || defined(_LINUX) || defined(__LINUX__)

#ifndef __LINUX__
#define __LINUX__
#endif

#ifndef __UNIX__
#define __UNIX__
#endif

#elif defined(macintosh) || defined(Macintosh) || defined(__APPLE__) || defined(__MACH__) || defined(MACOS) ||              \
    defined(MACOSX) || defined(__MACOS__)

#ifndef __MACOS__
#define __MACOS__
#endif

#ifndef __UNIX__
#define __UNIX__
#endif

#elif defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(WIN64) || defined(_WIN64) || defined(__WIN64__) || \
    defined(__TOS_WIN__) || defined(WINDOWS) || defined(_WINDOWS) || defined(__WINDOWS__)

#ifndef __WINDOWS__
#define __WINDOWS__
#endif

#else

#warning "Your operating system is not recognized."

#endif

