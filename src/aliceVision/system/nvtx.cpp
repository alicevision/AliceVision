#ifdef ALICEVISION_USE_NVTX

    #include <sstream>
    #include <filesystem>

    #if __has_include(<nvtx3/nvToolsExt.h>)
        #include <nvtx3/nvToolsExt.h>
    #elif __has_include(<nvToolsExtCuda.h>)
        #include <nvToolsExtCuda.h>
    #elif __has_include(<nvToolsExt.h>)
        #include <nvToolsExt.h>
    #else
        #error "ALICEVISION_USE_NVTX is enabled but no NVTX headers were found."
    #endif

    #include "aliceVision/system/nvtx.hpp"

void nvtxPushA(const char* label, const char* file, int line)
{
    std::filesystem::path in(file);

    std::ostringstream ostr;
    ostr << label << " " << in.filename() << ":" << line;
    nvtxRangePushA(ostr.str().c_str());
}

void nvtxPop(const char*) { nvtxRangePop(); }

#endif /* ALICEVISION_USE_NVTX */
