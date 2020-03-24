#include <aliceVision/image/all.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/mvsData/imageAlgo.hpp>
#include <aliceVision/image/drawing.hpp>

#include <random>

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

  bool appendImage(const image::Image<float> & grayscale_input) {

    if (grayscale_input.Width() != _source_width) {
      return false;
    }

    if (grayscale_input.Height() != _source_height) {
      return false;
    }

    /*Store pyramid for this image, will be processed later*/
    PyramidFloat pyramid(_source_width, _source_height, _minimal_size);
    if (!pyramid.apply(grayscale_input)) {
      return false;
    }

    _pyramids.push_back(pyramid);

    return true;
  }

  bool preprocessLevel(const PyramidFloat & pyramid, size_t level) {
    
    if (level >= pyramid.countLevels()) {
      return false;
    } 

    const image::Image<float> & source = pyramid.getLevel(level);

    /*Adapt current center to level*/
    double level_centerx = _center_x / pow(2.0, level);
    double level_centery = _center_y / pow(2.0, level);

    double max_rx = std::max(double(source.Width()) - level_centerx, level_centerx);
    double max_ry = std::max(double(source.Height()) - level_centery, level_centery);
    double max_radius = std::min(max_rx, max_ry);
    size_t max_radius_i = size_t(std::ceil(max_radius));
    double angles_bins = size_t(std::ceil(max_radius * M_PI)); 

    image::Image<float> polarImage(max_radius_i, angles_bins);
    if (!buildPolarImage(polarImage, source, level_centerx, level_centery)) {
      return false;
    }

    image::Image<float> gradientImage(max_radius_i, angles_bins);

    /*int max = pyramid.countLevels() - 1;*/
    /*int diff = max - level;*/
    int min_radius = 8;
    int radius = min_radius;// * pow(2, diff);
    
    if (!buildGradientImage(gradientImage, polarImage, radius)) {
      return false;
    }

    /*char filename[512];
    sprintf(filename, "/home/mmoc/grad%d.png", level);
    image::writeImage(filename, gradientImage, image::EImageColorSpace::SRGB);*/

    if (_gradientImage.Width() != gradientImage.Width() || _gradientImage.Height() != gradientImage.Height()) {
      _gradientImage = gradientImage;
    }
    else {
      _gradientImage += gradientImage;
    }

    return true;
  }

  bool process() {

    if (_pyramids.size() == 0) { 
      return false;
    }

    _center_x = _source_width / 2;
    _center_y = _source_height / 2;
    _radius = std::min(_source_width / 4, _source_height / 4);

    for (int current_level =  _pyramids[0].countLevels() - 1; current_level > 0; current_level--) {
      
      for (PyramidFloat & pyr : _pyramids) {
        if (!preprocessLevel(pyr, current_level)) {
          return false;
        }
      }
      
      int uncertainty = 50;
      if (current_level == _pyramids[0].countLevels() - 1) {
        uncertainty = std::max(_source_width, _source_height);
      }

      if (!processLevel(current_level, uncertainty)) {
        return false;
      }
    }

    return true;
  }

  bool processLevel(size_t level, int uncertainty) {

    const image::Image<float> gradients = _gradientImage;

    /*Adapt current center to level*/
    double level_centerx = _center_x / pow(2.0, level);
    double level_centery = _center_y / pow(2.0, level);
    double level_radius = _radius / pow(2.0, level);

    /* Extract maximas of response */
    std::vector<Eigen::Vector2d> selected_points;
    for (int y = 0; y < gradients.Height(); y++) {

      double rangle = double(y) * (2.0 * M_PI / double(gradients.Height()));
      double cangle = cos(rangle);
      double sangle = sin(rangle);

      double max_val = -1.0;
      int max_x = -1;

      /*Lookup possible radius*/
      int start = std::max(0, int(level_radius) - uncertainty);
      int end = std::min(gradients.Width() - 1, int(level_radius) + uncertainty);

      for (size_t x = start; x <= end; x++) {
        if (max_val < gradients(y, x)) {
          max_x = x;
          max_val = gradients(y, x);
        }
      }

      if (max_x > 0) {
        double nx = level_centerx + cangle * double(max_x);
        double ny = level_centery + sangle * double(max_x);
        selected_points.push_back({nx, ny});
      }
    }
    
    if (selected_points.size() < 3) {
      return false;
    }

    
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
      for (int i = 0; i < selected_points.size(); i++) {
        double cx = selected_points[i](0) - res(0);
        double cy = selected_points[i](1) - res(1);
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
        double dist = sqrt(cx * cx + cy * cy) - r;

        double w = 0.0;
        if (dist < c) {
          double xoc = dist / c;
          double hw = 1.0 - xoc * xoc;
          w = hw * hw;
        }

        Eigen::Vector3d J;
        if (std::abs(dist) < 1e-12) {
          J.fill(0);
          J(2) = -w;
        }
        else {
          J(0) = - w * cx / dist;
          J(1) = - w * cy / dist;
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

  bool buildPolarImage(image::Image<float> & dst, const image::Image<float> & src, float center_x, float center_y) {

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
        if (x < 0 || y < 0) continue;
        if (x >= src.Width() || y >= src.Height()) continue;
        dst(angle, amplitude) = sampler(src, y, x);
      }
    }
    
    return true;
  }

  bool buildGradientImage(image::Image<float> & dst, const image::Image<float> & src, size_t radius_size) {

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

        for (int dx = -kernel_radius; dx < 0; dx++) {
          sum_inside += src(angle, amplitude + dx);
        }
        for (int dx = 1; dx <= kernel_radius * 2; dx++) {
          sum_outside += src(angle, amplitude + dx);
        }

        dst(angle, amplitude) = sum_inside - sum_outside;
      }
    }
    
    return true;
  }

  /*void drawCircle(image::Image<image::RGBfColor> & dest) {
    image::DrawCircle(_center_x, _center_y, _radius, image::RGBfColor(1.0f), &dest);
  }*/

  double getCenterX() const {
    return _center_x;
  }

  double getCenterY() const {
    return _center_y;
  }

  double getRadius() const {
    return _radius;
  }

private:
  std::vector<PyramidFloat> _pyramids;
  image::Image<float> _gradientImage;
  
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

  std::string externalInfoFilename;
  std::string sfmInputDataFilename;
  std::string sfmOutputDataFilename;

  bool useFisheye = false;
  bool estimateFisheyeCircle = true;
  Vec2 fisheyeCenterOffset(0, 0);
  double fisheyeRadius = 96.0;

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());

  // Command line parameters
  po::options_description allParams(
    "Parse external information about cameras used in a panorama.\n"
    "AliceVision PanoramaInit");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmInputDataFilename)->required(), "SfMData file input.")
    ("outSfMDataFilename,o", po::value<std::string>(&sfmOutputDataFilename)->required(), "SfMData file output.")
    ;

  po::options_description motorizedHeadParams("Motorized Head parameters");
  motorizedHeadParams.add_options()
    ("config,c", po::value<std::string>(&externalInfoFilename), "External info xml file.")
    ;

  po::options_description fisheyeParams("Fisheye parameters");
  fisheyeParams.add_options()
    ("useFisheye", po::value<bool>(&useFisheye), "Declare all input images as fisheye with 'equidistant' model.")
    ("estimateFisheyeCircle", po::value<bool>(&estimateFisheyeCircle),
      "Automatically estimate the Fisheye Circle center and radius instead of using user values.")
    ("fisheyeCenterOffset_x", po::value<double>(&fisheyeCenterOffset(0)), "Fisheye circle's center offset X (pixels).")
    ("fisheyeCenterOffset_y", po::value<double>(&fisheyeCenterOffset(1)), "Fisheye circle's center offset Y (pixels).")
    ("fisheyeRadius,r", po::value<double>(&fisheyeRadius), "Fisheye circle's radius (% of image shortest side).")
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

    if(vm.count("help") || (argc == 1))
    {
      ALICEVISION_COUT(allParams);
      return EXIT_SUCCESS;
    }
    po::notify(vm);
  }
  catch(boost::program_options::required_option& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }
  catch(boost::program_options::error& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what());
    ALICEVISION_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }

  ALICEVISION_COUT("Program called with the following parameters:");
  ALICEVISION_COUT(vm);

  system::Logger::get()->setLogLevel(verboseLevel);

  sfmData::SfMData sfmData;
  if(!sfmDataIO::Load(sfmData, sfmInputDataFilename, sfmDataIO::ESfMData(sfmDataIO::ALL)))
  {
    ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilename << "' cannot be read.");
    return EXIT_FAILURE;
  }

  if(!externalInfoFilename.empty())
  {
    pt::ptree tree;

    try
    {
      pt::read_xml(externalInfoFilename, tree);
    }
    catch (...)
    {
      ALICEVISION_CERR("Error parsing input file");
    }

    pt::ptree lens = tree.get_child("papywizard.header.lens");
    pt::ptree shoot = tree.get_child("papywizard.shoot");

    std::string lensType = lens.get<std::string>("<xmlattr>.type");
    // double lensFocal = lens.get<double>("focal");

    /*Make sure we control everything for debug purpose*/
    if (lensType != "rectilinear")
    {
      ALICEVISION_CERR("Lens type not supported: " << lensType);
      return EXIT_FAILURE;
    }
    
    std::map<int, Eigen::Matrix3d> rotations;
    
    for (auto it : shoot)
    {
      int id = it.second.get<double>("<xmlattr>.id");
      int bracket = it.second.get<double>("<xmlattr>.bracket");

      if (rotations.find(id) != rotations.end())
      {
        ALICEVISION_CERR("Multiple xml attributes with a same id: " << id);
        return EXIT_FAILURE;
      }

      double yaw_degree = it.second.get<double>("position.<xmlattr>.yaw");
      double pitch_degree = it.second.get<double>("position.<xmlattr>.pitch");
      double roll_degree = it.second.get<double>("position.<xmlattr>.roll");

      double yaw = degreeToRadian(yaw_degree);
      double pitch = degreeToRadian(pitch_degree);
      double roll = degreeToRadian(roll_degree);

      Eigen::AngleAxis<double> Myaw(yaw, Eigen::Vector3d::UnitY());
      Eigen::AngleAxis<double> Mpitch(pitch, Eigen::Vector3d::UnitX());
      Eigen::AngleAxis<double> Mroll(roll, Eigen::Vector3d::UnitZ());
      Eigen::AngleAxis<double> Mimage(-M_PI_2, Eigen::Vector3d::UnitZ());

      Eigen::Matrix3d cRo = Myaw.toRotationMatrix() * Mpitch.toRotationMatrix() *  Mroll.toRotationMatrix() * Mimage.toRotationMatrix()  ;

      rotations[id] = cRo.transpose();
    }

    if (sfmData.getViews().size() != rotations.size())
    {
      ALICEVISION_LOG_ERROR("The input SfMData has not the same number of views than the config file (sfmData views:" << sfmData.getViews().size() << ", config file rotations: " << rotations.size() << ").");
      return EXIT_FAILURE; 
    }

    /**
     * HEURISTIC : 
     * The xml file describe rotations for view ids which are not correlated with Alicevision view id
     * Let assume that the order of xml views ids is the lexicographic order of the image names.
    */
    std::vector<std::pair<std::string, int>> names_with_id;
    for (auto v : sfmData.getViews())
    {
      names_with_id.push_back(std::make_pair(v.second->getImagePath(), v.first));
    }
    std::sort(names_with_id.begin(), names_with_id.end());

    size_t index = 0;
    for (auto &item_rotation: rotations)
    {
      IndexT viewIdx = names_with_id[index].second;

      if (item_rotation.second.trace() != 0)
      {
        sfmData::CameraPose pose(geometry::Pose3 (item_rotation.second, Eigen::Vector3d::Zero()));
        sfmData.setAbsolutePose(viewIdx, pose);
      }

      ++index;
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
        for(auto & v : sfmData.getViews()) {
          /*Read original image*/
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

        double cx = detector.getCenterX();
        double cy = detector.getCenterY();
        double r = detector.getRadius();

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

      equidistant->setCenterX(double(equidistant->w()) / 2.0 + fisheyeCenterOffset(0));
      equidistant->setCenterY(double(equidistant->h()) / 2.0 + fisheyeCenterOffset(1));

      equidistant->setRadius(fisheyeRadius / 100.0 * 0.5 * std::min(double(equidistant->w()),double(equidistant->h())));
      ++equidistantCount;
    }

    ALICEVISION_LOG_INFO(equidistantCount << " equidistant camera intrinsics have been updated");
  }

  if (!sfmDataIO::Save(sfmData, sfmOutputDataFilename, sfmDataIO::ESfMData(sfmDataIO::ALL)))
  {
    ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmOutputDataFilename << "' cannot be write.");
    return EXIT_FAILURE;
  }

  return 0;
}
