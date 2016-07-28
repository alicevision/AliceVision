#ifdef USE_OCVSIFT

#ifndef SIFT_OPENCV_IMAGE_DESCRIBER_HPP
#define	SIFT_OPENCV_IMAGE_DESCRIBER_HPP

/// Feature/Regions & Image describer interfaces
#include <openMVG/features/descriptor.hpp>
#include <openMVG/features/image_describer.hpp>
#include <openMVG/features/regions_factory.hpp>

#include <cereal/cereal.hpp>

/// OpenCV Includes
#include <opencv2/opencv.hpp>
#include "opencv2/core/eigen.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <chrono>

namespace openMVG {
namespace features {

class SIFT_OPENCV_Params
{
public:
  SIFT_OPENCV_Params() {}
  ~SIFT_OPENCV_Params() {}
  
  bool Set_configuration_preset(EDESCRIBER_PRESET preset)
  {
      switch(preset)
      {
        case LOW_PRESET:
          contrastThreshold = 0.01;
          maxTotalKeypoints = 1000;
          break;
        case MEDIUM_PRESET:
          contrastThreshold = 0.005;
          maxTotalKeypoints = 5000;
          break;
        case NORMAL_PRESET:
          contrastThreshold = 0.005;
          edgeThreshold = 15;
          maxTotalKeypoints = 10000;
          break;
        case HIGH_PRESET:
          contrastThreshold = 0.005;
          edgeThreshold = 20;
          maxTotalKeypoints = 20000;
          break;
        case ULTRA_PRESET:
          contrastThreshold = 0.005;
          edgeThreshold = 20;
          maxTotalKeypoints = 40000;
          break;
      }
      maxTotalKeypoints = 0; // TEMP, do not use GRID, line to delete
      return true;
  }

  template<class Archive>
  void serialize( Archive & ar )
  {
    ar(
      cereal::make_nvp("grid_size", gridSize),
      cereal::make_nvp("max_total_keypoints", maxTotalKeypoints),
      cereal::make_nvp("n_octave_layers", nOctaveLayers),
      cereal::make_nvp("contrast_threshold", contrastThreshold),
      cereal::make_nvp("edge_threshold", edgeThreshold),
      cereal::make_nvp("sigma", sigma));
      // cereal::make_nvp("root_sift", root_sift));
  }

  // Parameters
  std::size_t gridSize = 0; // TEMP, don't use GRID => 4
  std::size_t maxTotalKeypoints = 0; // TEMP, don't use GRID => 1000
  int nOctaveLayers = 3;  // default opencv value is 3
  double contrastThreshold = 0.04;  // default opencv value is 0.04
  double edgeThreshold = 10;
  double sigma = 1.6;
  // bool rootSift = true;
};

///
//- Create an Image_describer interface that use and OpenCV extraction method
// i.e. with the SIFT detector+descriptor
// Regions is the same as classic SIFT : 128 unsigned char
class SIFT_OPENCV_Image_describer : public Image_describer
{
public:
  SIFT_OPENCV_Image_describer() : Image_describer() {}

  ~SIFT_OPENCV_Image_describer() {}

  bool Set_configuration_preset(EDESCRIBER_PRESET preset)
  {
    return _params.Set_configuration_preset(preset);
  }
  /**
  @brief Detect regions on the image and compute their attributes (description)
  @param image Image.
  @param regions The detected regions and attributes (the caller must delete the allocated data)
  @param mask 8-bit gray image for keypoint filtering (optional).
     Non-zero values depict the region of interest.
  */
  bool Describe(const image::Image<unsigned char>& image,
    std::unique_ptr<Regions> &regions,
    const image::Image<unsigned char> * mask = NULL)
  {
    // Convert for opencv
    cv::Mat img;
    cv::eigen2cv(image.GetMat(), img);

    // Create a SIFT detector
    std::vector< cv::KeyPoint > v_keypoints;
    cv::Mat m_desc;
    std::size_t maxDetect = 0; // No max value by default
    if(_params.maxTotalKeypoints)
      if(!_params.gridSize)  // If no grid filtering, use opencv to limit the number of features
        maxDetect = _params.maxTotalKeypoints;

    //std::cout << "maxDetect: " << maxDetect << std::endl; // to clean
    std::cout << "First octave: " << "0 (hardcoded)" << std::endl;
    std::cout << "_params.nOctave: " << "8 (hardcoded)"  << std::endl;
    std::cout << "_params.nOctaveLayers: " << _params.nOctaveLayers << std::endl;
    std::cout << "_params.contrastThreshold: " << _params.contrastThreshold << std::endl;
    std::cout << "_params.edgeThreshold: " << _params.edgeThreshold << std::endl;
    std::cout << "_params.sigma: " << _params.sigma << std::endl;
    
    cv::Ptr<cv::Feature2D> siftdetector = cv::xfeatures2d::SIFT::create(maxDetect, _params.nOctaveLayers, _params.contrastThreshold, _params.edgeThreshold, _params.sigma);

    // Detect SIFT keypoints
    auto detect_start = std::chrono::steady_clock::now();
    siftdetector->detect(img, v_keypoints);
    auto detect_end = std::chrono::steady_clock::now();
    auto detect_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(detect_end - detect_start);

    //std::cout << "SIFT: contrastThreshold: " << _params.contrastThreshold << ", edgeThreshold: " << _params.edgeThreshold << std::endl;
    std::cout << "Detect SIFT: " << detect_elapsed.count() << " milliseconds." << std::endl;
    std::cout << "Image size: " << img.cols << " x " << img.rows << std::endl;
    std::cout << "Grid size: " << _params.gridSize << ", maxTotalKeypoints: " << _params.maxTotalKeypoints << std::endl;
    std::cout << "Number of detected features: " << v_keypoints.size() << std::endl;

    // cv::KeyPoint::response: the response by which the most strong keypoints have been selected.
    // Can be used for the further sorting or subsampling.
    std::sort(v_keypoints.begin(), v_keypoints.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return a.size > b.size; });

    // Grid filtering of the keypoints to ensure a global repartition
    if(_params.gridSize && _params.maxTotalKeypoints)
    {
      // Only filter features if we have more features than the maxTotalKeypoints
      if(v_keypoints.size() > _params.maxTotalKeypoints)
      {
        std::vector< cv::KeyPoint > filtered_keypoints;
        std::vector< cv::KeyPoint > rejected_keypoints;
        filtered_keypoints.reserve(std::min(v_keypoints.size(), _params.maxTotalKeypoints));
        rejected_keypoints.reserve(v_keypoints.size());

        cv::Mat countFeatPerCell(_params.gridSize, _params.gridSize, cv::DataType<std::size_t>::type, cv::Scalar(0));
        const std::size_t keypointsPerCell = _params.maxTotalKeypoints / countFeatPerCell.total();
        const double regionWidth = image.Width() / double(countFeatPerCell.cols);
        const double regionHeight = image.Height() / double(countFeatPerCell.rows);

        std::cout << "Grid filtering -- keypointsPerCell: " << keypointsPerCell
                  << ", regionWidth: " << regionWidth
                  << ", regionHeight: " << regionHeight << std::endl;

        for(const cv::KeyPoint& keypoint: v_keypoints)
        {
          const std::size_t cellX = std::min(std::size_t(keypoint.pt.x / regionWidth), _params.gridSize);
          const std::size_t cellY = std::min(std::size_t(keypoint.pt.y / regionHeight), _params.gridSize);
          // std::cout << "- keypoint.pt.x: " << keypoint.pt.x << ", keypoint.pt.y: " << keypoint.pt.y << std::endl;
          // std::cout << "- cellX: " << cellX << ", cellY: " << cellY << std::endl;
          // std::cout << "- countFeatPerCell: " << countFeatPerCell << std::endl;
          // std::cout << "- gridSize: " << _params.gridSize << std::endl;

          const std::size_t count = countFeatPerCell.at<std::size_t>(cellX, cellY);
          countFeatPerCell.at<std::size_t>(cellX, cellY) = count + 1;
          if(count < keypointsPerCell)
            filtered_keypoints.push_back(keypoint);
          else
            rejected_keypoints.push_back(keypoint);
        }
        // If we don't have enough features (less than maxTotalKeypoints) after the grid filtering (empty regions in the grid for example).
        // We add the best other ones, without repartition constraint.
        if( filtered_keypoints.size() < _params.maxTotalKeypoints )
        {
          const std::size_t remainingElements = std::min(rejected_keypoints.size(), _params.maxTotalKeypoints - filtered_keypoints.size());
          std::cout << "Grid filtering -- Copy remaining points: " << remainingElements << std::endl;
          filtered_keypoints.insert(filtered_keypoints.end(), rejected_keypoints.begin(), rejected_keypoints.begin() + remainingElements);
        }

        v_keypoints.swap(filtered_keypoints);
      }
    }
    std::cout << "Number of features: " << v_keypoints.size() << std::endl;

    // Compute SIFT descriptors
    auto desc_start = std::chrono::steady_clock::now();
    siftdetector->compute(img, v_keypoints, m_desc);
    auto desc_end = std::chrono::steady_clock::now();
    auto desc_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(desc_end - desc_start);
    std::cout << "Compute descriptors: " << desc_elapsed.count() << " milliseconds." << std::endl;

//    for(int i=0;i<m_desc.rows;++i)
//    {
//      std::cout << "New descriptor" << std::endl;
//      for(int k=0;k<128;++k)
//        std::cout << (int) m_desc.at<uchar>(i,k) << " ";
//    
//    std::cout << std::endl;
//    }
    
    Allocate(regions);

    // Build alias to cached data
    SIFT_Regions * regionsCasted = dynamic_cast<SIFT_Regions*>(regions.get());
    // reserve some memory for faster keypoint saving
    regionsCasted->Features().reserve(v_keypoints.size());
    regionsCasted->Descriptors().reserve(v_keypoints.size());

    // Prepare a column vector with the sum of each descriptor
    cv::Mat m_siftsum;
    cv::reduce(m_desc, m_siftsum, 1, cv::REDUCE_SUM);

    // Copy keypoints and descriptors in the regions
    int cpt = 0;
    for(std::vector< cv::KeyPoint >::const_iterator i_kp = v_keypoints.begin();
        i_kp != v_keypoints.end();
        ++i_kp, ++cpt)
    {
      SIOPointFeature feat((*i_kp).pt.x, (*i_kp).pt.y, (*i_kp).size, (*i_kp).angle);
      regionsCasted->Features().push_back(feat);

      Descriptor<unsigned char, 128> desc;
      for(int j = 0; j < 128; j++)
      {
        desc[j] = static_cast<unsigned char>(m_desc.at<float>(cpt, j));//512.0*sqrt(m_desc.at<float>(cpt, j)/m_siftsum.at<float>(cpt, 0)));
      }
      regionsCasted->Descriptors().push_back(desc);
    }

    return true;
  };

  /// Allocate Regions type depending of the Image_describer
  void Allocate(std::unique_ptr<Regions> &regions) const
  {
    regions.reset( new SIFT_Regions );
  }

  template<class Archive>
  void serialize( Archive & ar )
  {
    ar(cereal::make_nvp("params", _params));
  }

private:
  SIFT_OPENCV_Params _params;
};

} // namespace features
} // namespace openMVG

CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::features::SIFT_OPENCV_Image_describer, "SIFT_OPENCV_Image_describer");

#endif	/* SIFT_OPENCV_IMAGE_DESCRIBER_HPP */

#endif //USE_OCVSIFT

