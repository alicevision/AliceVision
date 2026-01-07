#include "sphereData.hpp"

#include <aliceVision/system/Logger.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace aliceVision {
namespace lightingEstimation {

CalibrationSphere::CalibrationSphere(const Eigen::Vector3f& center_, float radius_, bool is2D_, std::string type_)
  : center(center_),
    radius(radius_),
    is2d(is2D_),
    type(type_)
{

}

const Eigen::Vector3f& CalibrationSphere::getCenter() const { return center; }

float CalibrationSphere::getRadius() const { return radius; }

bool CalibrationSphere::is2D() const { return is2d; }

namespace CalibrationSphereIO {

namespace bpt = boost::property_tree;
bool load(CalibrationSpheres& calibrationSpheres, const std::string& filename)
{
    // Main tree
    bpt::ptree fileTree;
    // Read the json file and initialize the tree
    bpt::read_json(filename, fileTree);
    
    // iterator on spheres
    for (auto itview = fileTree.begin(); itview != fileTree.end(); itview++)
    {
		IndexT viewId = stoi(itview->first);
		auto calibrationSphereVec = std::make_shared<std::vector<CalibrationSphere>>();
        for (auto itsph = itview->second.begin(); itsph != itview->second.end(); itsph++)
        {
            float x = itsph->second.get_child("").get("x", 0.0);
            float y = itsph->second.get_child("").get("y", 0.0);
            float r = itsph->second.get_child("").get("r", 0.0);
            std::string type = itsph->second.get_child("").get("type", "matte");

            Eigen::Vector3f center;
            bool is2D;
            if (itsph->second.get_child("").count("z"))
            {
                float z = itsph->second.get_child("").get("z", 0.0);

                center << x, y, z;
                is2D = false;
            }
            else
            {
                center << x, y, 1.0;
                is2D = true;
            }
            CalibrationSphere calibrationSphere(center, r, is2D, type);
            calibrationSphereVec->push_back(calibrationSphere);
        }
		calibrationSpheres.emplace(viewId, calibrationSphereVec);
    }
    return true;
}
}

}  // namespace lightingEstimation
}  // namespace aliceVision
