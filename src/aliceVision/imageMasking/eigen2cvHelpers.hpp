#include "aliceVision/image/Image.hpp"
#include "aliceVision/image/pixelTypes.hpp"

#include <opencv2/core/core.hpp>


namespace cv{
template<> class DataType<aliceVision::image::RGBColor>
{
public:
    typedef uchar       value_type;
    typedef int         work_type;
    typedef value_type  channel_type;
    typedef value_type  vec_type;
    enum { generic_type = 0,
           depth        = CV_8U,
           channels     = 3,
           fmt          = (int)'u',
           type         = CV_MAKETYPE(depth, channels)
         };
};
}
