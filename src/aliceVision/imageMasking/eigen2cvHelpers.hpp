#include "aliceVision/image/Image.hpp"
#include "aliceVision/image/pixelTypes.hpp"

#include <opencv2/core/core.hpp>


namespace cv{
template<> class DataType<aliceVision::image::RGBColor>
{
public:
    using value_type = uchar;
    using work_type = int;
    using channel_type = value_type ;
    using vec_type = value_type;
    enum { generic_type = 0,
           depth        = CV_8U,
           channels     = 3,
           fmt          = (int)'u',
           type         = CV_MAKETYPE(depth, channels)
         };
};
}
