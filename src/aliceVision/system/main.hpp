// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// This one will be "main()" provided by the app
extern int aliceVision_main(int argc, char* argv[]);
/* This is the wrapper around aliceVision_main() (below).
 * Pass in function pointer to support shared linking. */
extern int aliceVision_main_wrapper(int(*realMain)(int, char*[]), int argc, char* argv[]);

// Inline to ensure the right main() is used (libf2c also has one...)
#if !defined(_ALICEVISION_SYSTEM_MAIN_IMPL)
int main(int argc, char* argv[])
{
  return aliceVision_main_wrapper(&aliceVision_main, argc, argv);
}
#endif // _ALICEVISION_SYSTEM_MAIN_IMPL

// This one will be "main()" provided by the app
#define main  aliceVision_main
