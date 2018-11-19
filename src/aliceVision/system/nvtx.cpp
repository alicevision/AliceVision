#ifdef ALICEVISION_USE_NVTX

#include <sstream>
#include <nvToolsExtCuda.h>
#include <boost/filesystem/path.hpp>

#include "aliceVision/system/nvtx.hpp"

void nvtxPushA( const char* label, const char* file, int line )
{
    boost::filesystem::path in( file );

    std::ostringstream ostr;
    ostr << label << " " << in.filename() << ":" << line;
    nvtxRangePushA( ostr.str().c_str() );
}

void nvtxPop ( const char* )
{
    nvtxRangePop( );
}

#endif /* ALICEVISION_USE_NVTX */

