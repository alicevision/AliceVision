#include "sift.hpp"

#include <openMVG/types.hpp>


namespace openMVG {
namespace features {


float getRadiusEstimate(const std::pair<size_t,size_t> & imgSize)
{
  return std::max(std::max(imgSize.first, imgSize.second) / float(600), 2.0f);
}

float getStrokeEstimate(const std::pair<size_t,size_t> & imgSize)
{
  return std::max(std::max(imgSize.first, imgSize.second) / float(2200), 2.0f);
}

/**
 * @brief Save a picture and its keypoints to an SVG file
 * @param inputImagePath - the path to the picture
 * @param imageSize - the size of the picture
 * @param keypoints - the keypoints (features) extracted from the picture
 * @param outputSVGPath - the path of the output SVG file
 * @param richKeypoint - do the keypoints contain orientation ?
 */
void tempSaveKeypoints2SVG(const std::string &inputImagePath,
                       const std::pair<size_t,size_t> & imageSize,
                       const std::vector<features::SIOPointFeature> &keypoints,
                       const std::string &outputSVGPath,
                       bool richKeypoint)
{
  svg::svgDrawer svgStream( imageSize.first, imageSize.second);
  svgStream.drawImage(inputImagePath, imageSize.first, imageSize.second);
  // heuristic for the radius of the feature according to the size of the image
  // the larger the distance the larger the minimum radius should be in order to be visible
  // We consider a minimum of 2 pixel and we increment it linearly according to 
  // the image size
  const float radius = getRadiusEstimate(imageSize);
  const float strokeWidth = getStrokeEstimate(imageSize);
  
  for(const features::SIOPointFeature &kpt : keypoints) 
  {
    svgStream.drawCircle(kpt.x(), kpt.y(), (richKeypoint) ? kpt.scale()*radius : radius, svg::svgStyle().stroke("yellow", strokeWidth));
    if(richKeypoint)
    {
      // compute the coordinate of the point on the circle used to draw the orientation line
      const float pointX = kpt.x()+std::cos(kpt.orientation())*kpt.scale()*radius;
      const float pointY = kpt.y()+std::sin(kpt.orientation())*kpt.scale()*radius;
      svgStream.drawLine(kpt.x(), kpt.y(), 
                         pointX, pointY,
                         svg::svgStyle().stroke("yellow", strokeWidth));
    }
  }
 
  std::ofstream svgFile( outputSVGPath );
  svgFile << svgStream.closeSvgFile().str();
  svgFile.close();  
}

/**
 * @brief Gives an histogram of the data (console)
 * Used for inter-descriptor distances (class are distances/1000)
 * @param dataVect - the vector containing the data
 */
void getStats(const std::vector<float>& dataVect)
{
  std::map<int,int> stats;

  for(int i = 0; i < dataVect.size(); ++i)
  {
    int currentData = std::floor(dataVect[i]/ 1000.0);
    if(stats.find(currentData) != stats.end())
    {
      stats[currentData] += 1;
    }
    else
    {
      stats[currentData] = 1;
    }
  }
  for(const auto& stat: stats)
  {
    std::cout << stat.first << "\t" << stat.second << std::endl;
  }
}



//convertSIFT
//////////////////////////////

/**
 * @brief Convert SIFT to a given type
 * @param descr - the vl_feat descriptor
 * @param descriptor - the output descriptor
 * @param brootSift - 
 */
template < typename TOut > 
inline void convertSIFT(
  const vl_sift_pix* descr,
  Descriptor<TOut,128> & descriptor,
  bool brootSift = false
  );

template <> 
inline void convertSIFT<float>(
  const vl_sift_pix* descr,
  Descriptor<float,128> &descriptor,
  bool brootSift)
{
  if(brootSift)
  {
    const float sum = std::accumulate(descr, descr + 128, 0.0f);
    for(int k = 0; k < 128; ++k)
      descriptor[k] = std::floor(512.f*sqrt(descr[k] / sum));
  }
  else
  {
    for(int k = 0; k < 128; ++k)
      descriptor[k] = std::floor(512.f*descr[k]);
  }
}

template <> 
inline void convertSIFT<unsigned char>(
  const vl_sift_pix* descr,
  Descriptor<unsigned char,128> & descriptor,
  bool brootSift)
{
  if (brootSift)
  {
    const float sum = std::accumulate(descr, descr+128, 0.0f);
    for (int k=0;k<128;++k)
      descriptor[k] = static_cast<unsigned char>(512.f*sqrt(descr[k]/sum));
  }
  else
  {
    for (int k=0;k<128;++k)
      descriptor[k] = static_cast<unsigned char>(512.f*descr[k]);
  }
}


/**
 * @brief Apply a grid filtering on the extracted keypoints
 * 
 * @param imageCV -  The input image (OpenCV format)
 * @parma twistedImage - The twisted output image (OpenCV format)
 * @parma tilt - The tilt parameter of ASIFT
 * @parma phi - The phi parameter of ASIFT
 * @parma mask -  8-bit gray image for keypoint filtering (optional).
 * @param finalTransformation - the affine transformation between the input and the output pictures
 */
template < typename T >
bool applyGrid(std::unique_ptr<Regions>& regions,
    const SiftParams& params,
    std::vector<VlSiftKeypoint>& vectKeyPoints,
    const int& w,
    const int& h)
{
  std::cout << "ApplyGrid" << std::endl; 
  typedef Scalar_Regions<SIOPointFeature,T,128> SIFT_Region_T; 
  // Build alias to cached data
  SIFT_Region_T * regionsCasted = dynamic_cast<SIFT_Region_T*>(regions.get());

  const auto& features = regionsCasted->Features();
  const auto& descriptors = regionsCasted->Descriptors();
  assert(features.size() == descriptors.size());
  assert(features.size() == vectKeyPoints.size());
  
  //Sorting the extracted features according to their scale
  {
    std::vector<std::size_t> indexSort(features.size());
    std::iota(indexSort.begin(), indexSort.end(), 0);
    std::sort(indexSort.begin(), indexSort.end(), [&](std::size_t a, std::size_t b){
      if(vectKeyPoints[a].o == vectKeyPoints[b].o)
        return vectKeyPoints[a].sigma > vectKeyPoints[b].sigma;
      return vectKeyPoints[a].o > vectKeyPoints[b].o;
    });

    std::vector<typename SIFT_Region_T::FeatureT> sortedFeatures(features.size());
    std::vector<typename SIFT_Region_T::DescriptorT> sortedDescriptors(features.size());
    std::vector<VlSiftKeypoint> sortedVectKeyPoints(features.size());
    
    for(std::size_t i: indexSort)
    {
      sortedFeatures[i] = features[indexSort[i]];
      sortedDescriptors[i] = descriptors[indexSort[i]];
      sortedVectKeyPoints[i] = vectKeyPoints[indexSort[i]];
    }

    regionsCasted->Features().swap(sortedFeatures);
    regionsCasted->Descriptors().swap(sortedDescriptors);
    sortedVectKeyPoints.swap(vectKeyPoints);
  }
  // Grid filtering of the keypoints to ensure a global repartition
  if(params._gridSize && params._maxTotalKeypoints)
  {
    // Only filter features if we have more features than the maxTotalKeypoints
    if(features.size() > params._maxTotalKeypoints)
    {
      std::vector<IndexT> filtered_indexes;
      std::vector<IndexT> rejected_indexes;
      filtered_indexes.reserve(std::min(features.size(), params._maxTotalKeypoints));
      rejected_indexes.reserve(features.size());

      const std::size_t sizeMat = params._gridSize * params._gridSize;
      std::vector<std::size_t> countFeatPerCell(sizeMat, 0);
      for (int Indice = 0; Indice < sizeMat; Indice++)
      {
    	  countFeatPerCell[Indice] = 0;
      }
      const std::size_t keypointsPerCell = params._maxTotalKeypoints / sizeMat;
      const double regionWidth = w / double(params._gridSize);
      const double regionHeight = h / double(params._gridSize);

      for(IndexT i = 0; i < features.size(); ++i)
      {
        const auto& keypoint = features.at(i);
        
        const std::size_t cellX = std::min(std::size_t(keypoint.x() / regionWidth), params._gridSize);
        const std::size_t cellY = std::min(std::size_t(keypoint.y() / regionHeight), params._gridSize);

        std::size_t &count = countFeatPerCell[cellX*params._gridSize + cellY];
        ++count;

        if(count < keypointsPerCell)
          filtered_indexes.push_back(i);
        else
          rejected_indexes.push_back(i);
      }
      // If we don't have enough features (less than maxTotalKeypoints) after the grid filtering (empty regions in the grid for example).
      // We add the best other ones, without repartition constraint.
      if( filtered_indexes.size() < params._maxTotalKeypoints )
      {
        const std::size_t remainingElements = std::min(rejected_indexes.size(), params._maxTotalKeypoints - filtered_indexes.size());
        std::cout << "Grid filtering -- Copy remaining points: " << remainingElements << std::endl;
        filtered_indexes.insert(filtered_indexes.end(), rejected_indexes.begin(), rejected_indexes.begin() + remainingElements);
      }

      std::vector<typename SIFT_Region_T::FeatureT> filtered_features(filtered_indexes.size());
      std::vector<typename SIFT_Region_T::DescriptorT> filtered_descriptors(filtered_indexes.size());
      std::vector<VlSiftKeypoint> filtered_VectKeyPoints(filtered_indexes.size());
      
      for(IndexT i = 0; i < filtered_indexes.size(); ++i)
      {
        filtered_features[i] = features[filtered_indexes[i]];
        filtered_descriptors[i] = descriptors[filtered_indexes[i]];
        filtered_VectKeyPoints[i] = vectKeyPoints[filtered_indexes[i]];
      }
      
      regionsCasted->Features().swap(filtered_features);
      regionsCasted->Descriptors().swap(filtered_descriptors);
      vectKeyPoints.swap(filtered_VectKeyPoints);      
 
    }
  }
  
  assert(features.size() == descriptors.size());
  assert(features.size() == vectKeyPoints.size());
  std::cout << "ApplyGrid end" << std::endl; 
  return true;
}


/**
 * @brief Standard SIFT extraction
 * 
 * @param[in] regions
 * @param[in] 
 * @param[in] 
 * @param[in] 
 * @param[in] 
 * @param[inout] filt
 */
template < typename T >
void extractFromFirstImage(std::unique_ptr<Regions> &regions,
    const SiftParams& params,
    const bool& bOrientation,
    const image::Image<unsigned char> * mask,
    const image::Image<float>& imageFloat,
    VlSiftFilt *filt,
    std::vector<VlSiftKeypoint>& vectKeyPoints)
{
  if (params._edge_threshold >= 0)
    vl_sift_set_edge_thresh(filt, params._edge_threshold);
  if (params._peak_threshold >= 0)
    vl_sift_set_peak_thresh(filt, 255.0 * params._peak_threshold/params._num_scales);
  
  vl_sift_process_first_octave(filt, imageFloat.data());
  
  Descriptor<vl_sift_pix, 128> vlFeatDescriptor;
  Descriptor<T, 128> descriptor;
  
  typedef Scalar_Regions<SIOPointFeature,T,128> SIFT_Region_T;
  regions.reset( new SIFT_Region_T );
  
  // Build alias to cached data
  SIFT_Region_T * regionsCasted = dynamic_cast<SIFT_Region_T*>(regions.get());
  
  // reserve some memory for faster keypoint saving
  const std::size_t reserveSize = (params._gridSize && params._maxTotalKeypoints) ? params._maxTotalKeypoints : 2000;
  regionsCasted->Features().reserve(reserveSize);
  regionsCasted->Descriptors().reserve(reserveSize);

  while (true)
  {
    vl_sift_detect(filt);

    const int nkeys = vl_sift_get_nkeypoints(filt);
    // Update gradient before launching parallel extraction
    vl_sift_update_gradient(filt);

    std::cout << std::endl;
    #ifdef OPENMVG_USE_OPENMP
    #pragma omp parallel for private(vlFeatDescriptor, descriptor)
    #endif
    for (int i = 0; i < nkeys; ++i)
    {
      // Feature masking
      if (mask)
      {
        const image::Image<unsigned char> & maskIma = *mask;
        if (maskIma(filt->keys[i].y, filt->keys[i].x) > 0)
          continue;
      }
      
      double angles [4] = {0.0, 0.0, 0.0, 0.0};
      int nangles = 1; // by default (1 upright feature)
      if (bOrientation)
      { // compute from 1 to 4 orientations
        nangles = vl_sift_calc_keypoint_orientations(filt, angles, &filt->keys[i]);
      }
      for (int q=0 ; q < nangles ; ++q)
      {
        vl_sift_calc_keypoint_descriptor(filt, &vlFeatDescriptor[0], &filt->keys[i], angles[q]);
        float xRound = filt->keys[i].x;
        float yRound = filt->keys[i].y;
        const SIOPointFeature fp(xRound, yRound,
          filt->keys[i].sigma, static_cast<float>(angles[q]));

        convertSIFT<T>(&vlFeatDescriptor[0], descriptor, params._root_sift);
        #ifdef OPENMVG_USE_OPENMP
        #pragma omp critical
        #endif
        {
          vectKeyPoints.push_back(filt->keys[i]);
          regionsCasted->Descriptors().push_back(descriptor);
          regionsCasted->Features().push_back(fp);
        }
      }
    }
    if (vl_sift_process_next_octave(filt))
      break; // Last octave
  }
}



template < typename T >
bool extractSIFT(const image::Image<unsigned char>& image,
    std::unique_ptr<Regions>& regions,
    const SiftParams& params,
    const bool bOrientation,
    const image::Image<unsigned char> * mask)
{
  const int w = image.Width(), h = image.Height();
  //Convert to float
  const image::Image<float> imageFloat(image.GetMat().cast<float>());

  std::vector<VlSiftKeypoint> vectKeyPoints;
  
  VlSiftFilt *mainFilt = vl_sift_new(w, h, params._num_octaves, params._num_scales, params._first_octave);
  extractFromFirstImage<T>(regions, params, bOrientation, mask, imageFloat, mainFilt, vectKeyPoints);
  std::cout << "FirstImage nb features: " << regions->RegionCount() << std::endl;
  applyGrid<T>(regions, params, vectKeyPoints, w, h);
  std::cout << "FirstImage nb features after Grid filtering: " << regions->RegionCount() << std::endl;

  return true;
}

template
bool extractSIFT<unsigned char>(const image::Image<unsigned char>& image,
    std::unique_ptr<Regions>& regions,
    const SiftParams& params,
    const bool bOrientation,
    const image::Image<unsigned char> * mask);

template
bool extractSIFT<float>(const image::Image<unsigned char>& image,
    std::unique_ptr<Regions>& regions,
    const SiftParams& params,
    const bool bOrientation,
    const image::Image<unsigned char> * mask);

}
}
