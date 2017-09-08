// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#ifdef _DEBUG
#define _CRTDBG_MAP_ALLOC
#include <crtdbg.h>
#include <stdlib.h>
//#define DEBUG_NEW new(_NORMAL_BLOCK, __FILE__, __LINE__)
//#define new DEBUG_NEW
#else
#include <stdlib.h>
#endif
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN // Exclude rarely-used stuff from Windows headers
#define NOMINMAX
#include <stdio.h>
#include <windows.h>
#include <tchar.h>
#endif

#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <stack>
#include <streambuf>
#include <string>
#include <time.h>
#include <vector>
