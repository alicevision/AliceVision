#include "sift.hpp"
#include <third_party/histogram/histogram.hpp>

#include <openMVG/types.hpp>


namespace openMVG {
namespace features {

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

/**
 * @brief Prints an histogram of the data (console) for a matrix
 * Used for inter-descriptor distances (class are distances/1000)
 * @param dataMatrix - the vector of vector containing the data
 */
void printMultiStats(const std::map<std::pair<float,float>, std::vector<float>>& dataMatrix)
{
  std::vector<Histogram<float>> histograms;

  for(auto& distPerVariation : dataMatrix)
  {
    Histogram<float> variationHistogram(0.0, 300.0, 100);
    variationHistogram.Add(distPerVariation.second.begin(), distPerVariation.second.end());

    histograms.emplace_back(variationHistogram);
  }
  std::cout << "\t";
  for(auto& distPerVariation : dataMatrix)
    std::cout << distPerVariation.first.first << "/" << distPerVariation.first.second << "\t";
  std::cout << std::endl;
  std::cout << multiHistogramToString(histograms) << std::endl;
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
 * @brief For a given region, creates a map with features positions as key and index as values
 * @param regions -  The detected regions and attributes
 * @param mapPairIndexes - the output map
 */
void getIndexMap(const std::unique_ptr<Regions>& regions,
                       std::map<std::pair<float,float>, std::vector<int> >& mapPairIndexes)
{
  for(int i = 0; i < regions->RegionCount(); i++)
  {
    const auto& feat = regions->GetRegionPosition(i);
    const std::pair<float,float> currentPair = std::make_pair(feat.x(), feat.y());
    mapPairIndexes[currentPair].push_back(i);
  }
}

/**
 * @brief For a given region, after ASIFT extraction, and a map with features positions as key and index as values,
 * computes distance between the descriptors from the twisted image and the corresponding descriptor from the input picture
 * and prints some statistics (console)
 * @param regions -  The detected regions and attributes
 * @param mapPairIndexes - the output map
 */
//std::map<(tilt,phi), std::vector<distance>> allDistances;
void retrieveASIFTDistances(const std::unique_ptr<Regions>& regions, std::map<std::pair<float,float>, std::vector<float>>& out_distances)
{
  std::map<std::pair<float,float>, std::vector<int> > mapPairIndexes;
  getIndexMap(regions, mapPairIndexes);

  const features::SIFT_Regions * regionsCasted = dynamic_cast<features::SIFT_Regions*>(regions.get());
  const auto& descriptors = regionsCasted->Descriptors();

  for(const auto& currentPairIndex : mapPairIndexes)
  {
    const std::vector<int>& currentIndexVector = currentPairIndex.second;

    // First descriptor from the original input image
    int siftFeatureIndex = currentIndexVector[0];
    const auto& refFeat = regionsCasted->Features()[siftFeatureIndex];
    assert(refFeat.tilt() == 0 && refFeat.phi() == 0);
    const auto& refDesc = descriptors[siftFeatureIndex];

    for(int i = 1; i < currentIndexVector.size(); ++i)
    {
      // Descriptor modified (tilt/phi)
      const auto& comparedDesc = descriptors[currentIndexVector[i]];
      float distance = voctree::L2<Descriptor<unsigned char, 128>>()(refDesc, comparedDesc);
      assert(distance >= 0);
      const auto& comparedFeat = regionsCasted->Features()[currentIndexVector[i]];
      out_distances[std::make_pair(comparedFeat.tilt(), comparedFeat.phi())].push_back(distance/1000.0);
    }
  }
}

void printStatisticsASIFT(const std::unique_ptr<Regions>& regions)
{
  std::map<std::pair<float,float>, std::vector<float>> distances;
  retrieveASIFTDistances(regions, distances);
  printMultiStats(distances);
}

/**
 * @brief Twists an image for the ASIFT extraction
 * @param imageCV -  The input image (OpenCV format)
 * @parma twistedImage - The twisted output image (OpenCV format)
 * @parma tilt - The tilt parameter of ASIFT
 * @parma phi - The phi parameter of ASIFT
 * @parma mask -  8-bit gray image for keypoint filtering (optional).
 * @param finalTransformation - the affine transformation between the input and the output pictures
 */
void getTwistedImage(const cv::Mat& imageCV,
        cv::Mat& twistedImage,
        double tilt,
        double phi,
        cv::Mat& mask,
        cv::Mat& finalTransformation)
{

  imageCV.copyTo(twistedImage);
  int h = imageCV.rows;
  int w = imageCV.cols;

  mask = cv::Mat(h, w, CV_8UC1, cv::Scalar(255));
  finalTransformation = cv::Mat::eye(2,3, CV_32F);

  if(phi != 0.0)
  {
    phi *= M_PI/180.;
    double s = sin(phi);
    double c = cos(phi);

    finalTransformation = (cv::Mat_<float>(2,2) << c, -s, s, c);

    cv::Mat corners = (cv::Mat_<float>(4,2) << 0, 0, w, 0, w, h, 0, h);
    cv::Mat tcorners = corners*finalTransformation.t();
    cv::Mat tcorners_x, tcorners_y;
    tcorners.col(0).copyTo(tcorners_x);
    tcorners.col(1).copyTo(tcorners_y);
    std::vector<cv::Mat> channels;
    channels.push_back(tcorners_x);
    channels.push_back(tcorners_y);
    merge(channels, tcorners);

    cv::Rect rect = cv::boundingRect(tcorners);
    finalTransformation =  (cv::Mat_<float>(2,3) << c, -s, -rect.x, s, c, -rect.y);

    cv::warpAffine(twistedImage, twistedImage, finalTransformation, cv::Size(rect.width, rect.height), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
  }
  if(tilt != 1.0)
  {
    double s = 0.8*sqrt(tilt*tilt-1);
    cv::GaussianBlur(twistedImage, twistedImage, cv::Size(0,0), s, 0.01);
    cv::resize(twistedImage, twistedImage, cv::Size(0,0), 1.0/tilt, 1.0, cv::INTER_NEAREST);
    finalTransformation.row(0) = finalTransformation.row(0)/tilt;
  }

  if(tilt != 1.0 || phi != 0.0)
  {
    h = twistedImage.rows;
    w = twistedImage.cols;
    cv::warpAffine(mask, mask, finalTransformation, cv::Size(w, h), cv::INTER_NEAREST);
  }

}

/**
 * @brief Apply a grid filtering on the extracted keypoints
 *

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
 * @param[in] regions The detected regions and attributes
 * @param[in] params The parameters of the Image describer
 * @param[in] bOrientation A boolean for SIFT orientation
 * @param[in] mask A binary mask (not used yet)
 * @param[in] imageFloat The image descriptors are extracted from
 * @param[inout] filt Used for vl_feat extraction
 * @param[inout] vectKeyPoints Keypoints extracted from the image
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



/**
 * @brief Extract descriptors from known keypoints for ASIFT.
 * @param[in] regions The detected regions and attributes
 * @param[in] params The parameters of the Image describer
 * @param[in] image The twisted image descriptors are extracted from
 * @param[in] mask A binary mask (not used yet)
 * @param[inout] filt Used for vl_feat extraction
 * @param[in] featuresPerOctave A map between octave and corresponding feature indexes
 * @param[in] finalTransformation The transformation between the original image and the twisted image
 * @param[inout] currentPictureKeypoints Keypoints extracted from the twisted image (for graphic purposes)
 * @param[in] tilt The ASIFT tilt parameter of the current twisted image 
 * @param[in] phi The ASIFT phi parameter of the current twisted image 
 */
template < typename T >
void extractFromTwistedImage(std::unique_ptr<Regions> &regions,
        const SiftParams& params,
        const image::Image<float>& image,
        const image::Image<unsigned char> * mask,
        VlSiftFilt *filt,
        const std::map<int, std::vector<VlSiftKeypoint>>& featuresPerOctave,
        const cv::Mat& finalTransformation,
        std::vector<features::SIOPointFeature>& currentPictureKeypoints,
        const float tilt,
        const float phi)
{
  //std::cout << "extractFromTwistedImage" << std::endl;
  if (params._edge_threshold >= 0)
    vl_sift_set_edge_thresh(filt, params._edge_threshold);
  if (params._peak_threshold >= 0)
    vl_sift_set_peak_thresh(filt, 255.0 * params._peak_threshold/params._num_scales);

  // Descriptors extraction
  vl_sift_process_first_octave(filt, image.data());
  Descriptor<vl_sift_pix, 128> vlFeatDescriptor;
  Descriptor<T, 128> descriptor;

  typedef Scalar_Regions<SIOPointFeature,T,128> SIFT_Region_T;

  // Build alias to cached data
  SIFT_Region_T * regionsCasted = dynamic_cast<SIFT_Region_T*>(regions.get());
  // reserve some memory for faster keypoint saving
  const std::size_t reserveSize = (params._gridSize && params._maxTotalKeypoints) ? params._maxTotalKeypoints : 2000;
  regionsCasted->Features().reserve(regionsCasted->Features().size() + reserveSize);
  regionsCasted->Descriptors().reserve(regionsCasted->Descriptors().size() + reserveSize);
  int cptOctaves = 0;


  float phiRad = phi * M_PI/180.;
  double s = sin(phiRad);
  double c = cos(phiRad);

  while(true)
  {
    vl_sift_detect(filt); // TODO: check if we can remove that

    // Update gradient before launching parallel extraction
    vl_sift_update_gradient(filt);


    if(featuresPerOctave.find(cptOctaves) != featuresPerOctave.end())
    {
      const std::vector<VlSiftKeypoint>& vectKeyPoints = featuresPerOctave.at(cptOctaves);
      const std::size_t nkeys = vectKeyPoints.size();

      filt->nkeys = nkeys;
      filt->keys_res = nkeys;
      filt->keys = (VlSiftKeypoint*) vl_realloc(filt->keys,
                              filt->keys_res * sizeof(VlSiftKeypoint));

      for (std::size_t i = 0; i < nkeys; ++i)
      {
        cv::Point3f twistedKeys(vectKeyPoints[i].x, vectKeyPoints[i].y, 1);
        cv::Mat twistedKeys_t = finalTransformation * cv::Mat(twistedKeys);
        filt->keys[i].x = twistedKeys_t.at<float>(0,0);
        filt->keys[i].y = twistedKeys_t.at<float>(1,0);
        filt->keys[i].sigma = vectKeyPoints[i].sigma;
        filt->keys[i].o = vectKeyPoints[i].o;
        filt->keys[i].s = vectKeyPoints[i].s;
        filt->keys[i].is = vectKeyPoints[i].is;
        filt->keys[i].ix = vectKeyPoints[i].ix;
        filt->keys[i].iy = vectKeyPoints[i].iy;
      }

      #ifdef OPENMVG_USE_OPENMP
      #pragma omp parallel for private(vlFeatDescriptor, descriptor)
      #endif
      for (int i = 0; i < nkeys; ++i)
      {
        // Feature masking
        if(mask)
        {
          const image::Image<unsigned char> & maskIma = *mask;
          if (maskIma(filt->keys[i].y, filt->keys[i].x) > 0)
            continue;
        }

        double angles [4] = {0.0, 0.0, 0.0, 0.0};
        int nangles; // by default (1 upright feature)
        nangles = vl_sift_calc_keypoint_orientations(filt, angles, &filt->keys[i]);
        nangles = 1; // Don't use multiple angles in ASIFT
        for (int q=0 ; q < nangles ; ++q)
        {
          vlFeatDescriptor.clear(0);
          vl_sift_calc_keypoint_descriptor(filt, &vlFeatDescriptor[0], &filt->keys[i], angles[q]);

          const SIOPointFeature fp(vectKeyPoints[i].x, vectKeyPoints[i].y,
                         filt->keys[i].sigma, angles[q], tilt, phi);

          SIOPointFeature currentPictureFp(filt->keys[i].x, filt->keys[i].y, filt->keys[i].sigma, angles[q]);
          convertSIFT<T>(&vlFeatDescriptor[0], descriptor, params._root_sift);
          #ifdef OPENMVG_USE_OPENMP
          #pragma omp critical
          #endif
          {
            regionsCasted->Descriptors().push_back(descriptor);
            regionsCasted->Features().push_back(fp);
            currentPictureKeypoints.push_back(currentPictureFp);
          }
        }
      }
    }
    ++cptOctaves;
    if (vl_sift_process_next_octave(filt))
      break; // Last octave
  }
}


/**
 * @brief Extract ASIFT regions (in float or unsigned char).
 * A first extraction is done using classic SIFT descriptors. The image is twisted in different ways.
 * Then, for each twist, for each feature, a new descriptor is extracted.
 * @param image - The input image
 * @param regions - The detected regions and attributes (the caller must delete the allocated data)
 * @param params - The parameters of the SIFT extractor
 * @param bOrientation - Compute orientation of SIFT descriptor (for the first extraction only)
 * @param mask - 8-bit gray image for keypoint filtering (optional).
 */
template < typename T >
bool extractASIFT(const image::Image<unsigned char>& image,
    std::unique_ptr<Regions>& regions,
    const ASiftParams& params,
    const bool bOrientation,
    const image::Image<unsigned char> * mask)
{
  const int w = image.Width(), h = image.Height();
  //Convert to float
  const image::Image<float> imageFloat(image.GetMat().cast<float>());

  std::vector<VlSiftKeypoint> vectKeyPoints;

  VlSiftFilt *mainFilt = vl_sift_new(w, h, params._num_octaves, params._num_scales, params._first_octave);
  extractFromFirstImage<T>(regions, params, bOrientation, mask, imageFloat, mainFilt, vectKeyPoints);
  applyGrid<T>(regions, params, vectKeyPoints, w, h);

  const int nbFeatures = vectKeyPoints.size();
  std::map<int, std::vector<VlSiftKeypoint>> featuresPerOctave;
  for(int i = 0; i < nbFeatures; ++i)
  {
    featuresPerOctave[vectKeyPoints[i].o].push_back(vectKeyPoints[i]);
  }
  vl_sift_delete(mainFilt);

  cv::Mat imgCV;
  cv::Mat maskCV; // not used yet
  cv::eigen2cv(imageFloat.GetMat(), imgCV);

  for(int tilt = 1; tilt <= params._nbTilts; tilt++)
  {
    double t;
    if(params._stepTilts)
    {
      t = 1 + tilt*params._stepTilts;
    }
    else
    {
      t = pow(2, 0.5*tilt);
    }

    for(double phi = 0; phi < 180; phi += 72.0/t)
    {
      cv::Mat twistedImageCV;
      cv::Mat finalTransformation;
      std::vector<features::SIOPointFeature> currentPictureKeypoints;

      getTwistedImage(imgCV, twistedImageCV, t, phi, maskCV, finalTransformation);

      image::Image<float> twistedImage(twistedImageCV.cols,twistedImageCV.rows);
      cv::cv2eigen(twistedImageCV, twistedImage);

      VlSiftFilt * twistedFilt = vl_sift_new(twistedImage.Width(), twistedImage.Height(), params._num_octaves, params._num_scales, params._first_octave);
      extractFromTwistedImage<T>(regions, params, twistedImage, mask, twistedFilt, featuresPerOctave, finalTransformation, currentPictureKeypoints, t, phi);

      vl_sift_delete(twistedFilt);
    }
  }


  return true;
}

template
bool extractASIFT<unsigned char>(const image::Image<unsigned char>& image,
    std::unique_ptr<Regions>& regions,
    const ASiftParams& params,
    const bool bOrientation,
    const image::Image<unsigned char> * mask);

template
bool extractASIFT<float>(const image::Image<unsigned char>& image,
    std::unique_ptr<Regions>& regions,
    const ASiftParams& params,
    const bool bOrientation,
    const image::Image<unsigned char> * mask);

/**
 * @brief Extract SIFT regions (in float or unsigned char).
 * Classic SIFT extraction
 * @param image - The input image
 * @param regions - The detected regions and attributes (the caller must delete the allocated data)
 * @param params - The parameters of the SIFT extractor
 * @param bOrientation - Compute orientation of SIFT descriptor (for the first extraction only)
 * @param mask - 8-bit gray image for keypoint filtering (optional).
 */
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
