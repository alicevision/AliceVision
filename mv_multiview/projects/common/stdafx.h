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
#include <tchar.h>
#include <windows.h>
#endif

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <sstream>
#include <stack>
#include <stdio.h>
#include <streambuf>
#include <string>
#include <time.h>
#include <vector>

//#pragma comment(lib,"psapi.lib")

// TODO: reference additional headers your program requires here

#include "structures/mv_common.h"
#include "structures/mv_multiview_params.h"
#include "structures/mv_structures.h"
