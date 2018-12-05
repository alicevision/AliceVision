#include <math.h>

#include "augmentedNormals.hpp"

namespace aliceVision {
namespace inverseRendering {

void getAugmentedNormals(AugmentedNormals& agmNormals, const Image<RGBfColor>& normals)
{
    using namespace Eigen;

    std::size_t nbPixels = normals.Width() * normals.Height();
    for(std::size_t y = 0; y < normals.Height(); ++y)
    {
        for(std::size_t x = 0; x < normals.Width(); ++x)
        {
            std::size_t i = y*normals.Width() + x;
            const RGBfColor& n = normals(y,x);
            agmNormals.nx(i) = n(0);
            agmNormals.ny(i) = n(1);
            agmNormals.nz(i) = n(2);
        }
    }

    agmNormals.ambiant.setOnes();

    agmNormals.nx_ny = agmNormals.nx*agmNormals.ny;
    agmNormals.nx_nz = agmNormals.nx*agmNormals.nz;
    agmNormals.ny_nz = agmNormals.ny*agmNormals.nz;
    agmNormals.nx2_ny2 = agmNormals.nx.cwiseAbs2() - agmNormals.ny.cwiseAbs2();
    agmNormals.nz2 = 3 * agmNormals.nz.cwiseAbs2() - agmNormals.ambiant;
}

}
}
