// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/config.hpp>
#include "distance.hpp"
#include "DefaultAllocator.hpp"

#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/feature/regionsFactory.hpp>

#include <aliceVision/types.hpp>
#include <aliceVision/system/Logger.hpp>

#include <stdint.h>
#include <vector>
#include <map>
#include <cassert>
#include <limits>
#include <fstream>
#include <stdexcept>
#include <iostream>


namespace aliceVision {
namespace voctree {

typedef int32_t Word;

typedef IndexT DocId;

typedef std::vector<Word> Document;
typedef std::map<Word, std::vector<IndexT> > SparseHistogram;
typedef std::map<DocId, SparseHistogram> SparseHistogramPerImage;

/**
 * Given a list of visual words associated to the features of a document it computes the 
 * vector of unique weighted visual words
 * 
 * @param[in] document a list of (possibly repeated) visual words
 * @param[out] v the vector of visual words (ie the visual word histogram of the image)
 */
inline void computeSparseHistogram(const std::vector<Word>& document, SparseHistogram& v)
{
  // for each visual word in the list
  for(std::size_t i = 0; i < document.size(); ++i)
  {
    // update its weighted count inside the map
    // the map v contains only the visual words that are associated to some features
    // the visual words in v are unique unlikely the document
    Word word = document[i];
    v[word].push_back(i);
  }
}

class IVocabularyTree
{
public:
  virtual ~IVocabularyTree() = 0;

  /// Save vocabulary to a file.
  virtual void save(const std::string& file) const = 0;
  /// Load vocabulary from a file.
  virtual void load(const std::string& file) = 0;

  /**
   * @brief Create a SparseHistogram from a blind vector of descriptors.
   * @param blindDescriptors
   * @return
   */
  virtual SparseHistogram quantizeToSparse(const void* blindDescriptors) const = 0;

  /// Get the depth (number of levels) of the tree.
  virtual uint32_t levels() const = 0;
  /// Get the branching factor (max splits at each node) of the tree.
  virtual uint32_t splits() const = 0;
  /// Get the number of words the tree contains.
  virtual uint32_t words() const = 0;

  /// Clears vocabulary, leaving an empty tree.
  virtual void clear() = 0;

};

inline IVocabularyTree::~IVocabularyTree() {}

/**
 * @brief Optimized vocabulary tree quantizer, templated on feature type and distance metric
 * for maximum efficiency.
 *
 * \c Feature is the data type of one feature. It has no requirements except compatibility with the distance metric.
 *
 * \c Distance is a functor that computes the distance between two Feature objects. It must have a \c result_type
 * typedef specifying the type of the returned distance. For the purposes of VocabularyTree, this need not even be
 * a metric; distances simply need to be comparable.
 *
 * \c FeatureAllocator is an STL-compatible allocator used to allocate Features internally.
 */
template<class Feature, template<typename, typename> class Distance = L2, // TODO: rename Feature into Descriptor
class FeatureAllocator = typename DefaultAllocator<Feature>::type>
class VocabularyTree : public IVocabularyTree
{
public:
  VocabularyTree();

  /**
   * @brief Create from vocabulary file.
   * @param file vocabulary file path
   */
  VocabularyTree(const std::string& file);

  /// Quantizes a feature into a discrete word.
  template<class DescriptorT>
  Word quantize(const DescriptorT& feature) const;

  /// Quantizes a set of features into visual words.
  template<class DescriptorT>
  std::vector<Word> quantize(const std::vector<DescriptorT>& features) const;

  /// Quantizes a set of features into sparse histogram of visual words.
  template<class DescriptorT>
  SparseHistogram quantizeToSparse(const std::vector<DescriptorT>& features) const;

  SparseHistogram quantizeToSparse(const void* blindDescriptors) const override
  {
    const std::vector<Feature>* descriptors = static_cast<const std::vector<Feature>*>(blindDescriptors);
    return quantizeToSparse(*descriptors);
  }

  /// Get the depth (number of levels) of the tree.
  uint32_t levels() const override;
  /// Get the branching factor (max splits at each node) of the tree.
  uint32_t splits() const override;
  /// Get the number of words the tree contains.
  uint32_t words() const override;

  /// Clears vocabulary, leaving an empty tree.
  void clear() override;

  /// Save vocabulary to a file.
  void save(const std::string& file) const override;
  /// Load vocabulary from a file.
  void load(const std::string& file) override;

  bool operator==(const VocabularyTree& other) const
  {
    return (centers_ == other.centers_) &&
        (valid_centers_ == other.valid_centers_) &&
        (k_ == other.k_) &&
        (levels_ == other.levels_) &&
        (num_words_ == other.num_words_) &&
        (word_start_ == other.word_start_);
  }

protected:
  std::vector<Feature, FeatureAllocator> centers_;
  std::vector<uint8_t> valid_centers_; /// @todo Consider bit-vector

  uint32_t k_; // splits, or branching factor
  uint32_t levels_;
  uint32_t num_words_; // number of leaf nodes
  uint32_t word_start_; // number of non-leaf nodes, or offset to the first leaf node

  bool initialized() const
  {
    return num_words_ != 0;
  }

  void setNodeCounts();
};

template<class Feature, template<typename, typename> class Distance, class FeatureAllocator>
VocabularyTree<Feature, Distance, FeatureAllocator>::VocabularyTree()
: k_(0), levels_(0), num_words_(0), word_start_(0)
{
}

template<class Feature, template<typename, typename> class Distance, class FeatureAllocator>
VocabularyTree<Feature, Distance, FeatureAllocator>::VocabularyTree(const std::string& file)
: k_(0), levels_(0), num_words_(0), word_start_(0)
{
  load(file);
}

template<class Feature, template<typename, typename> class Distance, class FeatureAllocator>
template<class DescriptorT>
Word VocabularyTree<Feature, Distance, FeatureAllocator>::quantize(const DescriptorT& feature) const
{
  typedef typename Distance<Feature, DescriptorT>::result_type distance_type;

  //	printf("asserting\n");
  assert(initialized());
  //	printf("initialized\n");
  int32_t index = -1; // virtual "root" index, which has no associated center.
  for(unsigned level = 0; level < levels_; ++level)
  {
    // Calculate the offset to the first child of the current index.
    int32_t first_child = (index + 1) * splits();
    // Find the child center closest to the query.
    int32_t best_child = first_child;
    distance_type best_distance = std::numeric_limits<distance_type>::max();
    for(int32_t child = first_child; child < first_child + (int32_t) splits(); ++child)
    {
      if(!valid_centers_[child])
        break; // Fewer than splits() children.
      distance_type child_distance = Distance<DescriptorT, Feature>()(feature, centers_[child]);
      if(child_distance < best_distance)
      {
        best_child = child;
        best_distance = child_distance;
      }
    }
    index = best_child;
  }

  return index - word_start_;
}

template<class Feature, template<typename, typename> class Distance, class FeatureAllocator>
template<class DescriptorT>
std::vector<Word> VocabularyTree<Feature, Distance, FeatureAllocator>::quantize(const std::vector<DescriptorT>& features) const
{
  // ALICEVISION_LOG_DEBUG("VocabularyTree quantize: " << features.size());
  std::vector<Word> imgVisualWords(features.size(), 0);

  // quantize the features
  #pragma omp parallel for
  for(ptrdiff_t j = 0; j < static_cast<ptrdiff_t>(features.size()); ++j)
  {
    // store the visual word associated to the feature in the temporary list
    imgVisualWords[j] = quantize<DescriptorT>(features[j]);
  }

  // add the vector to the documents
  return imgVisualWords;
}

template<class Feature, template<typename, typename> class Distance, class FeatureAllocator>
template<class DescriptorT>
SparseHistogram VocabularyTree<Feature, Distance, FeatureAllocator>::quantizeToSparse(const std::vector<DescriptorT>& features) const
{
  SparseHistogram histo;
  std::vector<Word> doc = quantize(features);
  computeSparseHistogram(doc, histo);
  return histo;
}

template<class Feature, template<typename, typename> class Distance, class FeatureAllocator>
uint32_t VocabularyTree<Feature, Distance, FeatureAllocator>::levels() const
{
  return levels_;
}

template<class Feature, template<typename, typename> class Distance, class FeatureAllocator>
uint32_t VocabularyTree<Feature, Distance, FeatureAllocator>::splits() const
{
  return k_;
}

template<class Feature, template<typename, typename> class Distance, class FeatureAllocator>
uint32_t VocabularyTree<Feature, Distance, FeatureAllocator>::words() const
{
  return num_words_;
}

template<class Feature, template<typename, typename> class Distance, class FeatureAllocator>
void VocabularyTree<Feature, Distance, FeatureAllocator>::clear()
{
  centers_.clear();
  valid_centers_.clear();
  k_ = levels_ = num_words_ = word_start_ = 0;
}

template<class Feature, template<typename, typename> class Distance, class FeatureAllocator>
void VocabularyTree<Feature, Distance, FeatureAllocator>::save(const std::string& file) const
{
  /// @todo Support serializing of non-"simple" feature classes
  /// @todo Some identifying name for the distance used
  assert(initialized());

  std::ofstream out(file.c_str(), std::ios_base::binary);
  out.write((char*) (&k_), sizeof (uint32_t));
  out.write((char*) (&levels_), sizeof (uint32_t));
  uint32_t size = centers_.size();
  out.write((char*) (&size), sizeof (uint32_t));
  out.write((char*) (&centers_[0]), centers_.size() * sizeof (Feature));
  out.write((char*) (&valid_centers_[0]), valid_centers_.size());
}

template<class Feature, template<typename, typename> class Distance, class FeatureAllocator>
void VocabularyTree<Feature, Distance, FeatureAllocator>::load(const std::string& file)
{
  clear();

  std::ifstream in;
  in.exceptions(std::ifstream::eofbit | std::ifstream::failbit | std::ifstream::badbit);

  uint32_t size;
  try
  {
    in.open(file.c_str(), std::ios_base::binary);
    in.read((char*) (&k_), sizeof (uint32_t));
    in.read((char*) (&levels_), sizeof (uint32_t));
    in.read((char*) (&size), sizeof (uint32_t));
    centers_.resize(size);
    valid_centers_.resize(size);
    in.read((char*) (&centers_[0]), centers_.size() * sizeof (Feature));
    in.read((char*) (&valid_centers_[0]), valid_centers_.size());
  }
  catch(std::ifstream::failure& e)
  {
    throw std::runtime_error("Failed to load vocabulary tree file" + file);
  }

  setNodeCounts();
  assert(size == num_words_ + word_start_);
}

template<class Feature, template<typename, typename> class Distance, class FeatureAllocator>
void VocabularyTree<Feature, Distance, FeatureAllocator>::setNodeCounts()
{
  num_words_ = k_;
  word_start_ = 0;
  for(uint32_t i = 0; i < levels_ - 1; ++i)
  {
    word_start_ += num_words_;
    num_words_ *= k_;
  }
}

/**
 * @brief compute the sparse distance between two histograms according to the chosen distance method.
 * 
 * @param v1 The first sparse histogram
 * @param v2 The second sparse histogram
 * @param distanceMethod distance method (norm L1, etc.)
 * @param word_weights 
 * @return the distance of the two histograms
 */
float sparseDistance(const SparseHistogram& v1, const SparseHistogram& v2, const std::string &distanceMethod = "classic", const std::vector<float>& word_weights = std::vector<float>());

inline std::unique_ptr<IVocabularyTree> createVoctreeForDescriberType(feature::EImageDescriberType imageDescriberType)
{
  using namespace aliceVision::feature;
  std::unique_ptr<IVocabularyTree> res;

  switch(imageDescriberType)
  {
    case EImageDescriberType::SIFT:       res.reset(new VocabularyTree<SIFT_Regions::DescriptorT>); break;
    case EImageDescriberType::SIFT_FLOAT: res.reset(new VocabularyTree<SIFT_Float_Regions::DescriptorT>); break;
    case EImageDescriberType::AKAZE:      res.reset(new VocabularyTree<AKAZE_Float_Regions::DescriptorT>); break;
    case EImageDescriberType::AKAZE_MLDB: res.reset(new VocabularyTree<AKAZE_BinaryRegions::DescriptorT>); break;

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
    case EImageDescriberType::CCTAG3:
    case EImageDescriberType::CCTAG4:     res.reset(new VocabularyTree<CCTAG_Regions::DescriptorT>); break;
#endif //ALICEVISION_HAVE_CCTAG

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OCVSIFT)
  case EImageDescriberType::SIFT_OCV:     res.reset(new VocabularyTree<SIFT_Regions::DescriptorT>); break;
#endif //ALICEVISION_HAVE_OCVSIFT
  case EImageDescriberType::AKAZE_OCV:    res.reset(new VocabularyTree<AKAZE_Float_Regions::DescriptorT>); break;
#endif //ALICEVISION_HAVE_OPENCV

    default: throw std::out_of_range("Invalid imageDescriber enum");
  }

  return res;
}

inline void load(std::unique_ptr<IVocabularyTree>& out_voctree, feature::EImageDescriberType& out_descType, const std::string& filepath)
{
  std::size_t lastDot = filepath.find_last_of(".");
  if(lastDot == std::string::npos)
    throw std::invalid_argument("Unrecognized Vocabulary tree filename (no extension): " + filepath);
  std::size_t secondLastDot = filepath.find_last_of(".", lastDot-1);
  if(secondLastDot == std::string::npos)
    throw std::invalid_argument("Unrecognized Vocabulary tree filename (no descType in extension): " + filepath);

  const std::string descTypeStr = filepath.substr((secondLastDot+1), lastDot - (secondLastDot+1));

  out_descType = feature::EImageDescriberType_stringToEnum(descTypeStr);
  out_voctree = createVoctreeForDescriberType(out_descType);
  out_voctree->load(filepath);
}

}
}
