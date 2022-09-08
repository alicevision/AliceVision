#include "Undistortion.hpp"

#include "Undistortion3DE.hpp"

namespace aliceVision{
namespace camera{

std::shared_ptr<Undistortion> Undistortion::create(const Type& type, int width, int height)
{
    if (type == Type::ANAMORPHIC4)
    {
        return std::shared_ptr<Undistortion>(new Undistortion3DEAnamorphic4(width, height));
    } 
     
    return nullptr;
}

}
}