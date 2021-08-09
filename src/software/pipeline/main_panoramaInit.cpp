#include <aliceVision/image/all.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/mvsData/imageAlgo.hpp>
#include <aliceVision/image/drawing.hpp>

#include <random>
#include <algorithm>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 2
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace std {
std::ostream& operator<<(std::ostream& os, const std::pair<double, double>& v)
{
    os << v.first << " " << v.second;
    return os;
}

std::istream& operator>>(std::istream& in, std::pair<double, double>& v)
{
    std::string token;
    in >> token;
    v.first = boost::lexical_cast<double>(token);
    in >> token;
    v.second = boost::lexical_cast<double>(token);
    return in;
}
}

namespace po = boost::program_options;
namespace fs = boost::filesystem;
namespace pt = boost::property_tree;


/**
 * A simple class for gaussian pyramid
 */
class PyramidFloat {
public:
  PyramidFloat(size_t width, size_t height, size_t minimal_size) {
    
    /*minimal_size * 2^n = minside*/
    size_t minside = std::min(width, height);
    double scale = std::log2(double(minside)/double(minimal_size));
    int levels = std::floor(scale);
    
    size_t cwidth = width;
    size_t cheight = height;
    for (int i = 0; i <= levels; i++) {

      _levels.push_back(image::Image<float>(cwidth, cheight));

      cheight /= 2;
      cwidth /= 2;
    }
  }

  bool apply(const image::Image<float> & grayscale_input) {
    
    /*First of all, build pyramid for filtering high frequencies*/
    _levels[0] = grayscale_input;
    for (int level = 1; level < _levels.size(); level++)  {
      
      size_t sw = _levels[level - 1].Width();
      size_t sh = _levels[level - 1].Height();
      size_t dw = _levels[level].Width();
      size_t dh = _levels[level].Height();

      oiio::ImageSpec spec_src(sw, sh, 1, oiio::TypeDesc::FLOAT);
      oiio::ImageSpec spec_dst(dw, dh, 1, oiio::TypeDesc::FLOAT);

      oiio::ImageBuf buf_src(spec_src, const_cast<float*>(_levels[level - 1].data()));
      oiio::ImageBuf buf_dst(spec_dst, const_cast<float*>(_levels[level].data()));
      
      oiio::ImageBufAlgo::resize(buf_dst, buf_src, "gaussian");
    }

    return true;
  }

  size_t countLevels() const {
    return _levels.size();
  }

  const image::Image<float> & getLevel(size_t level) const {
    return _levels[level];
  }

private:
  std::vector<image::Image<float>> _levels;
};

class CircleDetector {
public:

  CircleDetector() = delete;

  CircleDetector(size_t width, size_t height, size_t minimal_size)
  : _source_width(width), _source_height(height), _minimal_size(minimal_size), _radius(0) {
  }

  void setDebugDirectory(const std::string& dir) {
    _debugDirectory = dir;
  }

  bool appendImage(const image::Image<float> & grayscale_input) {

    if (grayscale_input.Width() != _source_width) {
      return false;
    }

    if (grayscale_input.Height() != _source_height) {
      return false;
    }

    /*
    Store pyramid for this image, will be processed later
    This way we ensure we do not loose intermediate information
    */
    PyramidFloat pyramid(_source_width, _source_height, _minimal_size);
    if (!pyramid.apply(grayscale_input)) {
      return false;
    }
    _pyramids.push_back(pyramid);

    return true;
  }

  bool preprocessLevel(const PyramidFloat& pyramid, size_t pyramid_id, size_t level)
  {    
    if (level >= pyramid.countLevels()) {
      return false;
    } 

    const image::Image<float> & source = pyramid.getLevel(level);

    /*Adapt current center to level*/
    double level_centerx = _center_x / pow(2.0, level);
    double level_centery = _center_y / pow(2.0, level);

    /* Compute the maximal distance between the borders and the circle center */
    double max_rx = std::max(double(source.Width()) - level_centerx, level_centerx);
    double max_ry = std::max(double(source.Height()) - level_centery, level_centery);

    //Just in case the smallest side is cropped inside the circle.
    double max_radius = std::min(max_rx, max_ry);
    max_radius *= 1.5;

    size_t max_radius_i = size_t(std::ceil(max_radius));
    double angles_bins = size_t(std::ceil(max_radius * M_PI)); 

    /* Build a polar image using the estimated circle center */
    image::Image<float> polarImage(max_radius_i, angles_bins, true, 0.0f);
    image::Image<unsigned char> polarImageMask(max_radius_i, angles_bins, true, 0);
    if (!buildPolarImage(polarImage, polarImageMask, source, level_centerx, level_centery)) {
      return false;
    }
    
    debugImage(polarImage, "polarImage", pyramid_id, level);

    /* Use a custom edge detector */
    /*int max = pyramid.countLevels() - 1;*/
    /*int diff = max - level;*/
    int min_radius = 8;
    int radius = min_radius;// * pow(2, diff);
    image::Image<float> gradientImage(max_radius_i, angles_bins);    
    if (!buildGradientImage(gradientImage, polarImage, polarImageMask, radius)) {
      return false;
    }

    debugImage(gradientImage, "gradientImage", pyramid_id, level);

    if (_gradientImage.Width() != gradientImage.Width() || _gradientImage.Height() != gradientImage.Height()) {
      _gradientImage = gradientImage;
    }
    else {
      _gradientImage += gradientImage;
    }

    return true;
  }

  bool process()
  {
    if (_pyramids.empty())
    { 
      return false;
    }

    /* Initialize parameters with most common case */
    _center_x = _source_width / 2;
    _center_y = _source_height / 2;
    _radius = std::min(_source_width / 4, _source_height / 4);
    size_t last_level_inliers = 0;
    int last_valid_level = -1;

    for (int current_level =  _pyramids[0].countLevels() - 1; current_level > 1; current_level--)
    {
      /* Compute gradients */
      size_t current_pyramid_id = 0;
      for (PyramidFloat & pyr : _pyramids)
      {
        if(!preprocessLevel(pyr, current_pyramid_id, current_level))
        {
          return false;
        }
        current_pyramid_id++;
      }
      
      /* Estimate the search area */
      int uncertainty = 50;
      if (current_level == _pyramids[0].countLevels() - 1)
      {
        uncertainty = std::max(_source_width, _source_height);
      }

      debugImage(_gradientImage, "globalGradientImage", 0, current_level);

      /* Perform estimation */
      if(!processLevel(current_level, uncertainty, last_level_inliers))
      {
        break;
      }

      last_valid_level = current_level;
    }

    /*Check that the circle was detected at some level*/
    if (last_valid_level < 0)
    {
      return false;
    }

    return true;
  }

  bool processLevel(size_t level, int uncertainty, size_t& last_level_inliers)
  {
    image::Image<float> gradients = _gradientImage;
    
    image::Image<unsigned char> selection(gradients.Width(), gradients.Height(), true, 0);

    /*Adapt current center to level*/
    double level_centerx = _center_x / pow(2.0, level);
    double level_centery = _center_y / pow(2.0, level);
    double level_radius = _radius / pow(2.0, level);
    int min_radius = gradients.Width() / 2;
    

    /* Extract maximas of response */
    std::vector<Eigen::Vector2d> selected_points;
    for (int y = 0; y < gradients.Height(); y++) {

      double rangle = double(y) * (2.0 * M_PI / double(gradients.Height()));
      double cangle = cos(rangle);
      double sangle = sin(rangle);

      double max_val = -1.0;
      int max_x = -1;

      /*Lookup possible radius*/
      int start = std::max(min_radius, int(level_radius) - uncertainty);
      int end = std::min(gradients.Width() - 1, int(level_radius) + uncertainty);

      /*Remove non maximas*/
      for (int x = start; x < end; x++) {
        if (gradients(y, x) < gradients(y, x + 1)) {
          gradients(y, x) = 0.0f;
        }
      }

      /*Remove non maximas*/
      for (int x = end; x > start; x--) {
        if (gradients(y, x) < gradients(y, x - 1)) {
          gradients(y, x) = 0.0f;
        }
      }

      /* Store maximas */
      for (int x = start; x <= end; x++) {
        if (gradients(y, x) > 0.0f) {
          double nx = level_centerx + cangle * double(x);
          double ny = level_centery + sangle * double(x);
          selected_points.push_back({nx, ny});
          selection(y, x) = 255;
        }
      }
    }
    
    if (selected_points.size() < 3) {
      return false;
    }

    debugImage(selection, "selected", 0, level);

    /***
    * RANSAC
    */
    std::default_random_engine generator;
    std::uniform_int_distribution<int> distribution(0, selected_points.size() - 1);

    size_t maxcount = 0;
    Eigen::Vector3d best_params;
    for (int i = 0; i < 10000; i++) {

      int id1 = distribution(generator);
      int id2 = distribution(generator);
      int id3 = distribution(generator);
        
      if (id1 == id2 || id1 == id3 || id2 == id3) continue;

      Eigen::Vector2d p1 = selected_points[id1];
      Eigen::Vector2d p2 = selected_points[id2];
      Eigen::Vector2d p3 = selected_points[id3];

      Eigen::Vector3d res;
      if (!fitCircle(res, p1, p2, p3)) {
        continue;
      }

      size_t count = 0;
      for(const auto& point : selected_points)
      {
        double cx = point(0) - res(0);
        double cy = point(1) - res(1);
        double r = res(2);

        double dist = std::abs(sqrt(cx * cx + cy * cy) - r);
        if (dist < 3) {
          count++;
        }
      }

      if (count > maxcount) {
        maxcount = count;
        best_params = res;
      }
    } 

    if (maxcount < last_level_inliers) {
      return false;
    }
    last_level_inliers = maxcount;

    /***
    * Minimize
    */
    double sigma = 2.0;
    double c = sigma * 4.6851;
    Eigen::Vector3d previous_good = best_params;
    double last_good_error = std::numeric_limits<double>::max();

    for (int iter = 0; iter < 1000; iter++) {
      
      double sum_error = 0.0;
      for (int i = 0; i < selected_points.size(); i++) {
        double cx = selected_points[i](0) - best_params(0);
        double cy = selected_points[i](1) - best_params(1);
        double r = best_params(2);
        double dist = pow(sqrt(cx * cx + cy * cy) - r, 2.0);

        double w = 0.0;
        if (dist < c) {
          double xoc = dist / c;
          double hw = 1.0 - xoc * xoc;
          w = hw * hw;
        }

        sum_error += w * dist;
      }

      if (sum_error > last_good_error) {
        best_params = previous_good;
        break;
      }

      last_good_error = sum_error;

      Eigen::Matrix3d JtJ = Eigen::Matrix3d::Zero();
      Eigen::Vector3d Jte = Eigen::Vector3d::Zero();
      for (auto & pt : selected_points) {

        double cx = pt(0) - best_params(0);
        double cy = pt(1) - best_params(1);
        double r = best_params(2);
        double normsq = cx * cx + cy * cy;
        double norm = sqrt(normsq);
        double dist = norm - r;

        

        double w = 0.0;
        if (dist < c) {
          double xoc = dist / c;
          double hw = 1.0 - xoc * xoc;
          w = hw * hw;
        }

        Eigen::Vector3d J;
        if (std::abs(normsq) < 1e-12) {
          J.fill(0);
          J(2) = -w;
        }
        else {
          J(0) = - w * cx / norm;
          J(1) = - w * cy / norm;
          J(2) = - w;
        }

        for (int i = 0; i < 3; i++) {
          for (int j = 0; j < 3; j++) {
            JtJ(i, j) += J(i) * J(j);
          }

          Jte(i) += J(i) * dist;
        }
      }

      previous_good = best_params;
      best_params -= JtJ.inverse() * Jte;
    }

    _center_x = best_params(0) * pow(2.0, level);
    _center_y = best_params(1) * pow(2.0, level);
    _radius = best_params(2) * pow(2.0, level);

    return true;
  }

  bool fitCircle(Eigen::Vector3d & output, const Eigen::Vector2d & pt1, const Eigen::Vector2d & pt2, const Eigen::Vector2d & pt3) {

    /**
    Solve :
    (pt1x - centerx)^2 + (pt1y - centery)^2 - r^2 = 0
    (pt2x - centerx)^2 + (pt2y - centery)^2 - r^2 = 0
    (pt3x - centerx)^2 + (pt3y - centery)^2 - r^2 = 0

    -----
    l1 : pt1x^2 + centerx^2 - 2*pt1x*centerx + pt1y^2 + centery^2 - 2*pt1y*centery - r^2 = 0
    l2 : pt2x^2 + centerx^2 - 2*pt2x*centerx + pt2y^2 + centery^2 - 2*pt2y*centery - r^2 = 0
    l3 : pt3x^2 + centerx^2 - 2*pt3x*centerx + pt3y^2 + centery^2 - 2*pt3y*centery - r^2 = 0
    -----
    l2 - l1 : (pt2x^2 + centerx^2 - 2*pt2x*centerx + pt2y^2 + centery^2 - 2*pt2y*centery - r^2) - (pt1x^2 + centerx^2 - 2*pt1x*centerx + pt1y^2 + centery^2 - 2*pt1y*centery - r^2) = 0
    l3 - l1 : (pt3x^2 + centerx^2 - 2*pt3x*centerx + pt3y^2 + centery^2 - 2*pt3y*centery - r^2) - (pt1x^2 + centerx^2 - 2*pt1x*centerx + pt1y^2 + centery^2 - 2*pt1y*centery - r^2) = 0
    -----
    l2 - l1 : pt2x^2 + centerx^2 - 2*pt2x*centerx + pt2y^2 + centery^2 - 2*pt2y*centery - r^2 - pt1x^2 - centerx^2 + 2*pt1x*centerx - pt1y^2 - centery^2 + 2*pt1y*centery + r^2) = 0
    l3 - l1 : pt3x^2 + centerx^2 - 2*pt3x*centerx + pt3y^2 + centery^2 - 2*pt3y*centery - r^2 - pt1x^2 - centerx^2 + 2*pt1x*centerx - pt1y^2 - centery^2 + 2*pt1y*centery + r^2) = 0
    -----
    l2 - l1 : pt2x^2 - 2*pt2x*centerx + pt2y^2 - 2*pt2y*centery - pt1x^2 + 2*pt1x*centerx - pt1y^2 + 2*pt1y*centery = 0
    l3 - l1 : pt3x^2 - 2*pt3x*centerx + pt3y^2 - 2*pt3y*centery - pt1x^2 + 2*pt1x*centerx - pt1y^2 + 2*pt1y*centery = 0
    -----
    l2 - l1 : pt2x^2 + pt2y^2 - pt1x^2 - pt1y^2 + 2*pt1x*centerx - 2*pt2x*centerx + 2*pt1y*centery - 2*pt2y*centery = 0
    l3 - l1 : pt3x^2 + pt3y^2 - pt1x^2 - pt1y^2 - 2*pt3x*centerx - 2*pt3y*centery + 2*pt1x*centerx + 2*pt1y*centery = 0
    -----
    l2 - l1 : pt2x^2 + pt2y^2 - pt1x^2 - pt1y^2 + 2*(pt1x - pt2x)*centerx + 2*(pt1y-pt2y)*centery = 0
    l3 - l1 : pt3x^2 + pt3y^2 - pt1x^2 - pt1y^2 + 2*(pt1x - pt3x)*centerx + 2*(pt1y-pt3y)*centery = 0
    -----
    l2 - l1 : A + C*centerx + E*centery = 0
    l3 - l1 : B + D*centerx + F*centery = 0
    -----
    l2 - l1 : centerx = - (A + E * centery) / C
    l3 - l1 : B + D * centerx + F * centery = 0
    -----
    l2 - l1 : centerx = G + H * centery;
    l3 - l1 : B + D * (G + H * centery) + F * centery = 0
    -----
    l2 - l1 : centerx = G + H * centery;
    l3 - l1 : B + D * G + D * H * centery + F * centery = 0
    -----
    l2 - l1 : centerx = G + H * centery;
    l3 - l1 : B + D * G + (D * H + F) * centery = 0
    -----
    l2 - l1 : centerx = G + H * centery;
    l3 - l1 : centery = - (B + D * G) / (D * H + F);

    ----
    l1 : (pt1x - centerx)^2 + (pt1y - centery)^2 = r^2
    */

    double A = pt2(0)*pt2(0) + pt2(1)*pt2(1) - pt1(0)*pt1(0) - pt1(1)*pt1(1);
    double B = pt3(0)*pt3(0) + pt3(1)*pt3(1) - pt1(0)*pt1(0) - pt1(1)*pt1(1);
    double C = 2.0 * (pt1(0) - pt2(0));
    double D = 2.0 * (pt1(0) - pt3(0));
    double E = 2.0 * (pt1(1) - pt2(1));
    double F = 2.0 * (pt1(1) - pt3(1));
    if (std::abs(C) < 1e-12) return false;

    double G = - A / C;
    double H = - E / C;

    if (std::abs(D* H + F) < 1e-12) return false;

    double centery = - (B + D * G) / (D* H + F);
    double centerx = G + H * centery;

    output(0) = centerx;
    output(1) = centery;
    output(2) = sqrt((pt1(0) - centerx)*(pt1(0) - centerx) + (pt1(1) - centery)*(pt1(1) - centery));

    return true;
  }

  bool buildPolarImage(image::Image<float> & dst, image::Image<unsigned char> & dstmask, const image::Image<float> & src, float center_x, float center_y) {

    image::Sampler2d<image::SamplerLinear> sampler;
    size_t count_angles = dst.Height();


    for (int angle = 0; angle < count_angles; angle++) {
      double rangle = angle * (2.0 * M_PI / double(count_angles));
      double cangle = cos(rangle);
      double sangle = sin(rangle);

      for (int amplitude = 0; amplitude < dst.Width(); amplitude++) {

        double x = center_x + cangle * double(amplitude);
        double y = center_y + sangle * double(amplitude);

        dst(angle, amplitude) = 0;
        dstmask(angle, amplitude) = 0;

        if (x < 0 || y < 0) continue;
        if (x >= src.Width() || y >= src.Height()) continue;
        dst(angle, amplitude) = sampler(src, y, x);
        dstmask(angle, amplitude) = 255;
      }
    }
    
    return true;
  }

  bool buildGradientImage(image::Image<float> & dst, const image::Image<float> & src, const image::Image<unsigned char> & srcMask, size_t radius_size) {

    /*Build gradient for x coordinates image */
    dst.fill(0);

    int kernel_radius = radius_size;
    for (int angle = 0; angle < src.Height(); angle++) {
      
      int start = radius_size;
      int end = src.Width() - kernel_radius * 2;

      for (int amplitude = start; amplitude < end; amplitude++) {
        dst(angle, amplitude) = 0.0;

        float sum_inside = 0.0;
        float sum_outside = 0.0;

        unsigned char valid = 255;

        for (int dx = -kernel_radius; dx < 0; dx++) {
          sum_inside += src(angle, amplitude + dx);
          valid &= srcMask(angle, amplitude + dx);
        }
        for (int dx = 1; dx <= kernel_radius * 2; dx++) {
          sum_outside += src(angle, amplitude + dx);
          valid &= srcMask(angle, amplitude + dx);
        }

        if (valid) {
          dst(angle, amplitude) = std::max(0.0f, (sum_inside - sum_outside));
        }
        else {
          dst(angle, amplitude) = 0.0f;
        }
      }
    }
    
    return true;
  }

  double getCircleCenterX() const {
    return _center_x;
  }

  double getCircleCenterY() const {
    return _center_y;
  }

  double getCircleRadius() const {
    return _radius;
  }

  template <class T>
  void debugImage(const image::Image<T> & toSave, const std::string & name, int pyramid_id, int level) {
    // Only export debug image if there is an debug output folder defined.
    if(_debugDirectory.empty())
      return;

    boost::filesystem::path filepath = boost::filesystem::path(_debugDirectory) /
          (name + "_" + std::to_string(pyramid_id) + "_" + std::to_string(level) + ".exr");
    image::writeImage(filepath.string(), toSave, image::EImageColorSpace::AUTO);
  }

private:
  std::vector<PyramidFloat> _pyramids;
  image::Image<float> _gradientImage;
  std::string _debugDirectory;
  
  double _center_x;
  double _center_y;
  double _radius;

  size_t _source_width;
  size_t _source_height; 
  size_t _minimal_size;
};

int main(int argc, char * argv[])
{
    using namespace aliceVision;

    std::string externalInfoFilepath;
    std::string sfmInputDataFilepath;
    std::string sfmOutputDataFilepath;
    std::string inputAngleString;
    std::string initializeCameras;
    std::string nbViewsPerLineString;

    bool yawCW = true;
    bool useFisheye = false;
    bool estimateFisheyeCircle = true;
    Vec2 fisheyeCenterOffset(0, 0);
    double fisheyeRadius = 96.0;
    float additionalAngle = 0.0f;
    bool debugFisheyeCircleEstimation = false;

    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());

    // Command line parameters
    po::options_description allParams(
        "Parse external information about cameras used in a panorama.\n"
        "AliceVision PanoramaInit");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmInputDataFilepath)->required(), "SfMData file input.")
    ("outSfMData,o", po::value<std::string>(&sfmOutputDataFilepath)->required(), "SfMData file output.")
    ;

    po::options_description motorizedHeadParams("Motorized Head parameters");
    motorizedHeadParams.add_options()
    ("config,c", po::value<std::string>(&externalInfoFilepath), "External info xml file from a motorized head system.")
    ("inputAngle,a", po::value<std::string>(&inputAngleString), "External info xml additional angle.")
    ("yawCW", po::value<bool>(&yawCW), "Yaw rotation is ClockWise or ConterClockWise.")
    ("initializeCameras", po::value<std::string>(&initializeCameras), "Initialization type for the cameras poses.")
    ("nbViewsPerLine", po::value<std::string>(&nbViewsPerLineString), "Number of views per line splitted by comma. For instance, \"2,4,*,4,2\".")
    ;

    po::options_description fisheyeParams("Fisheye parameters");
    fisheyeParams.add_options()
    ("useFisheye", po::value<bool>(&useFisheye), "Declare all input images as fisheye with 'equidistant' model.")
    ("estimateFisheyeCircle", po::value<bool>(&estimateFisheyeCircle),
        "Automatically estimate the Fisheye Circle center and radius instead of using user values.")
        ("fisheyeCenterOffset_x", po::value<double>(&fisheyeCenterOffset(0)), "Fisheye circle's center offset X (pixels).")
        ("fisheyeCenterOffset_y", po::value<double>(&fisheyeCenterOffset(1)), "Fisheye circle's center offset Y (pixels).")
        ("fisheyeRadius,r", po::value<double>(&fisheyeRadius), "Fisheye circle's radius (% of image shortest side).")
        ("debugFisheyeCircleEstimation", po::value<bool>(&debugFisheyeCircleEstimation),
            "Debug fisheye circle detection.")
        ;

    po::options_description optionalParams("Optional parameters");

    po::options_description logParams("Log parameters");
    logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
        "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(motorizedHeadParams).add(fisheyeParams).add(logParams);

    // Parse command line
    po::variables_map vm;
    try
    {
        po::store(po::parse_command_line(argc, argv, allParams), vm);

        if (vm.count("help") || (argc == 1))
        {
            ALICEVISION_COUT(allParams);
            return EXIT_SUCCESS;
        }
        po::notify(vm);
    }
    catch (boost::program_options::required_option& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }
    catch (boost::program_options::error& e)
    {
        ALICEVISION_CERR("ERROR: " << e.what());
        ALICEVISION_COUT("Usage:\n\n" << allParams);
        return EXIT_FAILURE;
    }

    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT(vm);

    system::Logger::get()->setLogLevel(verboseLevel);

    if (inputAngleString == "rotate90")
    {
        additionalAngle = -M_PI_2;
    }
    else if (inputAngleString == "rotate180")
    {
        additionalAngle = -M_PI;
    }
    else if (inputAngleString == "rotate270")
    {
        additionalAngle = M_PI_2;
    }

    sfmData::SfMData sfmData;
    if (!sfmDataIO::Load(sfmData, sfmInputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    {
        // Setup known poses from XML file or user expression
        std::map<int, Eigen::Matrix3d> rotations;

        boost::to_lower(initializeCameras);
        if(initializeCameras == "no")
        {
        }
        else if(initializeCameras == "file" || (initializeCameras.empty() && !externalInfoFilepath.empty()))
        {
            if(externalInfoFilepath.empty())
            {
                ALICEVISION_LOG_ERROR("Init cameras from file, but path is not set.");
                return EXIT_FAILURE;
            }

            pt::ptree tree;

            try
            {
                pt::read_xml(externalInfoFilepath, tree);
            }
            catch(...)
            {
                ALICEVISION_CERR("Error parsing input file");
                return EXIT_FAILURE;
            }

            pt::ptree shoot = tree.get_child("papywizard.shoot");
            for(auto it : shoot)
            {
                int id = it.second.get<double>("<xmlattr>.id");
                int bracket = it.second.get<double>("<xmlattr>.bracket");

                if(rotations.find(id) != rotations.end())
                {
                    ALICEVISION_CERR("Multiple xml attributes with a same id: " << id);
                    return EXIT_FAILURE;
                }

                const double yaw_degree = it.second.get<double>("position.<xmlattr>.yaw");
                const double pitch_degree = it.second.get<double>("position.<xmlattr>.pitch");
                const double roll_degree = it.second.get<double>("position.<xmlattr>.roll");

                const double yaw = degreeToRadian(yaw_degree);
                const double pitch = degreeToRadian(pitch_degree);
                const double roll = degreeToRadian(roll_degree);

                const Eigen::AngleAxis<double> Myaw(yaw, Eigen::Vector3d::UnitY());
                const Eigen::AngleAxis<double> Mpitch(pitch, Eigen::Vector3d::UnitX());
                const Eigen::AngleAxis<double> Mroll(roll, Eigen::Vector3d::UnitZ());
                const Eigen::AngleAxis<double> Mimage(additionalAngle - M_PI_2, Eigen::Vector3d::UnitZ());

                const Eigen::Matrix3d cRo = Myaw.toRotationMatrix() * Mpitch.toRotationMatrix() *
                                            Mroll.toRotationMatrix() * Mimage.toRotationMatrix();

                rotations[id] = cRo.transpose();
            }

            if(sfmData.getViews().size() != rotations.size())
            {
                ALICEVISION_LOG_ERROR(
                    "The input SfMData has not the same number of views than the config file (sfmData views:"
                    << sfmData.getViews().size() << ", config file rotations: " << rotations.size() << ").");
                return EXIT_FAILURE;
            }
        }
        else if(boost::algorithm::contains(initializeCameras, "horizontal"))
        {
            const double zenithPitch = -0.5 * boost::math::constants::pi<double>();
            const Eigen::AngleAxis<double> zenithMpitch(zenithPitch, Eigen::Vector3d::UnitX());
            const Eigen::AngleAxis<double> zenithMroll(additionalAngle, Eigen::Vector3d::UnitZ());
            const Eigen::Matrix3d zenithRo = zenithMpitch.toRotationMatrix() * zenithMroll.toRotationMatrix();

            const bool withZenith = boost::algorithm::contains(initializeCameras, "zenith");
            if(initializeCameras == "zenith+horizontal")
            {
                ALICEVISION_LOG_TRACE("Add zenith first");
                rotations[rotations.size()] = zenithRo.transpose();
            }
            const std::size_t nbHorizontalViews = sfmData.getViews().size() - int(withZenith);
            for(int x = 0; x < nbHorizontalViews; ++x)
            {
                double yaw = 0;
                if(nbHorizontalViews > 1)
                {
                    // Vary horizontally between -180 and +180 deg
                    yaw = (yawCW ? 1.0 : -1.0) * x * 2.0 * boost::math::constants::pi<double>() / double(nbHorizontalViews);
                }

                Eigen::AngleAxis<double> Myaw(yaw, Eigen::Vector3d::UnitY());
                Eigen::AngleAxis<double> Mroll(additionalAngle, Eigen::Vector3d::UnitZ());

                Eigen::Matrix3d cRo =
                    Myaw.toRotationMatrix() * Mroll.toRotationMatrix();

                ALICEVISION_LOG_TRACE("Add rotation: yaw=" << yaw);
                rotations[rotations.size()] = cRo.transpose();
            }
            if(initializeCameras == "horizontal+zenith")
            {
                ALICEVISION_LOG_TRACE("Add zenith");
                rotations[rotations.size()] = zenithRo.transpose();
            }
        }
        else if(initializeCameras == "spherical" || (initializeCameras.empty() && !nbViewsPerLineString.empty()))
        {
            if(nbViewsPerLineString.empty())
            {
                ALICEVISION_LOG_ERROR("Init cameras from Sperical, but 'nbViewsPerLine' is not set.");
                return EXIT_FAILURE;
            }

            std::vector<std::string> nbViewsStrPerLine;
            boost::split(nbViewsStrPerLine, nbViewsPerLineString, boost::is_any_of(", "));
            const int totalNbViews = sfmData.getViews().size();
            std::vector<int> nbViewsPerLine;
            int nbAutoSize = 0;
            int sum = 0;
            for(const std::string& nbViewsStr : nbViewsStrPerLine)
            {
                if(nbViewsStr == "*")
                {
                    nbViewsPerLine.push_back(-1);
                    ++nbAutoSize;
                }
                else
                {
                    int v = boost::lexical_cast<int>(nbViewsStr);
                    nbViewsPerLine.push_back(v);
                    if(v == -1)
                    {
                        ++nbAutoSize;
                    }
                    else
                    {
                        sum += v;
                    }
                }
            }
            if(sum > totalNbViews)
            {
                ALICEVISION_LOG_ERROR("The input SfMData has less cameras declared than the number of cameras declared "
                                      "in the expression (sfmData views:"
                                      << sfmData.getViews().size() << ", expression sum: " << sum << ").");
                return EXIT_FAILURE;
            }
            if(nbAutoSize > 0)
            {
                std::replace(nbViewsPerLine.begin(), nbViewsPerLine.end(), -1, (totalNbViews - sum) / nbAutoSize);
            }
            if(nbViewsPerLine.empty())
            {
                // If no expression assume that it is a pure rotation around one axis
                nbViewsPerLine.push_back(totalNbViews);
            }
            const std::size_t newSum = std::accumulate(nbViewsPerLine.begin(), nbViewsPerLine.end(), 0);

            if(newSum != totalNbViews)
            {
                ALICEVISION_LOG_ERROR(
                    "The number of cameras in the input SfMData does not match with the number of cameras declared "
                    "in the expression (sfmData views:"
                    << sfmData.getViews().size() << ", expression sum: " << newSum << ").");
                return EXIT_FAILURE;
            }

            int i = 0;
            for(int y = 0; y < nbViewsPerLine.size(); ++y)
            {
                double pitch = 0;
                if(nbViewsPerLine.size() > 1)
                {
                    // Vary vertically between -90 and +90 deg
                    pitch = (-0.5 * boost::math::constants::pi<double>()) +
                          y * boost::math::constants::pi<double>() / double(nbViewsPerLine.size());
                }

                const int nbViews = nbViewsPerLine[y];
                for(int x = 0; x < nbViews; ++x)
                {
                    double yaw = 0;
                    if(nbViews > 1)
                    {
                        // Vary horizontally between -180 and +180 deg
                        yaw = (yawCW ? 1.0 : -1.0) * x * 2.0 * boost::math::constants::pi<double>() / double(nbViews);
                    }
                    const double roll = 0;

                    Eigen::AngleAxis<double> Myaw(yaw, Eigen::Vector3d::UnitY());
                    Eigen::AngleAxis<double> Mpitch(pitch, Eigen::Vector3d::UnitX());
                    Eigen::AngleAxis<double> Mroll(roll + additionalAngle, Eigen::Vector3d::UnitZ());

                    Eigen::Matrix3d cRo = Myaw.toRotationMatrix() * Mpitch.toRotationMatrix() *
                                          Mroll.toRotationMatrix();

                    ALICEVISION_LOG_TRACE("Add rotation: yaw=" << yaw << ", pitch=" << pitch << ", roll=" << roll
                                                               << ".");
                    rotations[i++] = cRo.transpose();
                }
            }
        }


        if(!rotations.empty())
        {
            ALICEVISION_LOG_TRACE("Apply rotations from nbViewsPerLine expressions: " << nbViewsPerLineString << ".");

            if(rotations.size() != sfmData.getViews().size())
            {
                ALICEVISION_LOG_ERROR("The number of cameras in the input SfMData does not match with the number of "
                                      "rotations to apply (sfmData nb views:"
                                      << sfmData.getViews().size() << ", nb rotations: " << rotations.size() << ").");
                return EXIT_FAILURE;
            }
            /**
             * HEURISTIC :
             * The xml file describe rotations for view ids which are not correlated with Alicevision view id
             * Let assume that the order of xml views ids is the lexicographic order of the image names.
             */
            std::vector<std::pair<std::string, int>> names_with_id;
            for(auto v : sfmData.getViews())
            {
                boost::filesystem::path path_image(v.second->getImagePath());
                names_with_id.push_back(std::make_pair(path_image.stem().string(), v.first));
            }
            std::sort(names_with_id.begin(), names_with_id.end());

            size_t index = 0;
            for(auto& item_rotation : rotations)
            {
                IndexT viewIdx = names_with_id[index].second;
                if(item_rotation.second.trace() != 0)
                {
                    sfmData::CameraPose pose(geometry::Pose3(item_rotation.second, Eigen::Vector3d::Zero()));
                    sfmData.setAbsolutePose(viewIdx, pose);
                }
                ++index;
            }
        }
    }

  if(useFisheye)
  {
      sfmData::Intrinsics & intrinsics = sfmData.getIntrinsics();
      for (auto & intrinsic_pair : intrinsics)
      {
        std::shared_ptr<camera::IntrinsicBase>& intrinsic = intrinsic_pair.second;
        std::shared_ptr<camera::IntrinsicsScaleOffset> intrinsicSO = std::dynamic_pointer_cast<camera::IntrinsicsScaleOffset>(intrinsic);
        std::shared_ptr<camera::EquiDistantRadialK3> equidistant = std::dynamic_pointer_cast<camera::EquiDistantRadialK3>(intrinsic);

        if (intrinsicSO != nullptr && equidistant == nullptr)
        {
            ALICEVISION_LOG_INFO("Replace intrinsic " << intrinsic_pair.first << " of type " << intrinsic->getTypeStr() <<  " to an EquiDistant camera model.");
            // convert non-EquiDistant intrinsics to EquiDistant
            std::shared_ptr<camera::EquiDistantRadialK3> newEquidistant(new camera::EquiDistantRadialK3());

            newEquidistant->copyFrom(*intrinsicSO);
            // "radius" and "center" will be set later from the input parameters in another loop

            // replace the intrinsic
            intrinsic = newEquidistant;
        }
      }
  }

  {
    int equidistantCount = 0;

    if(useFisheye && estimateFisheyeCircle)
    {
      
      if(sfmData.getIntrinsics().size() != 1)
      {
        ALICEVISION_LOG_ERROR("Only one intrinsic allowed (" << sfmData.getIntrinsics().size() << " found)");
        return EXIT_FAILURE;
      }

      std::shared_ptr<camera::IntrinsicBase> intrinsic = sfmData.getIntrinsics().begin()->second;
      if(intrinsic == nullptr)
      {
        ALICEVISION_LOG_ERROR("No valid intrinsic");
        return EXIT_FAILURE;
      }

      if(camera::isEquidistant(intrinsic->getType()))
      { 
        CircleDetector detector(intrinsic->w(), intrinsic->h(), 256);
        if(debugFisheyeCircleEstimation)
        {
            boost::filesystem::path path(sfmOutputDataFilepath);
            detector.setDebugDirectory(path.parent_path().string());
        }
        for(auto & v : sfmData.getViews())
        {
          // Read original image
          image::Image<float> grayscale;
          image::readImage(v.second->getImagePath(), grayscale, image::EImageColorSpace::SRGB);

          bool res = detector.appendImage(grayscale);
          if(!res)
          {
            ALICEVISION_LOG_ERROR("Image is incompatible with fisheye detection");
            return EXIT_FAILURE;
          }
        }

        if(!detector.process())
        {
          ALICEVISION_LOG_ERROR("Failed to find circle");
          return EXIT_FAILURE;
        }

        double cx = detector.getCircleCenterX();
        double cy = detector.getCircleCenterY();
        double r = detector.getCircleRadius();

        // Update parameters with estimated values
        fisheyeCenterOffset(0) = cx - 0.5*double(intrinsic->w());
        fisheyeCenterOffset(1) = cy - 0.5*double(intrinsic->h());
        fisheyeRadius = 98.0 * r / (0.5 * std::min(double(intrinsic->w()), double(intrinsic->h())));

        ALICEVISION_LOG_INFO("Computing automatic fisheye circle");
        ALICEVISION_LOG_INFO(" * Center Offset: " << fisheyeCenterOffset);
        ALICEVISION_LOG_INFO(" * Radius: " << fisheyeRadius);
      }
    }

    sfmData::Intrinsics & intrinsics = sfmData.getIntrinsics();
    for (auto & intrinsic_pair : intrinsics)
    {
      std::shared_ptr<camera::IntrinsicBase> intrinsic = intrinsic_pair.second;
      std::shared_ptr<camera::EquiDistant> equidistant = std::dynamic_pointer_cast<camera::EquiDistant>(intrinsic);
      if (equidistant == nullptr)
      {
        // skip non equidistant cameras
        continue;
      }
      ALICEVISION_LOG_INFO("Update EquiDistant camera intrinsic " << intrinsic_pair.first << " with center and offset.");

      equidistant->setCircleCenterX(double(equidistant->w()) / 2.0 + fisheyeCenterOffset(0));
      equidistant->setCircleCenterY(double(equidistant->h()) / 2.0 + fisheyeCenterOffset(1));

      equidistant->setCircleRadius(fisheyeRadius / 100.0 * 0.5 * std::min(double(equidistant->w()),double(equidistant->h())));
      ++equidistantCount;
    }

    ALICEVISION_LOG_INFO(equidistantCount << " equidistant camera intrinsics have been updated");
  }

  ALICEVISION_LOG_INFO("Export SfM: " << sfmOutputDataFilepath);
  if(!sfmDataIO::Save(sfmData, sfmOutputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
  {
    ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmOutputDataFilepath << "' cannot be write.");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
