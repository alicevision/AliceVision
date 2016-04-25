#ifndef OPENMVG_VOCABULARY_TREE_VOCABULARY_TREE_HPP
#define OPENMVG_VOCABULARY_TREE_VOCABULARY_TREE_HPP

#include "distance.hpp"
#include "feature_allocator.hpp"

#include <openMVG/types.hpp>

#include <stdint.h>
#include <vector>
#include <map>
#include <cassert>
#include <limits>
#include <fstream>
#include <stdexcept>
#include <iostream>


namespace openMVG {
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

/**
 * @brief Optimized vocabulary tree quantizer, templated on feature type and distance metric
 * for maximum efficiency.
 *
 * \c VoctreeDescriptorT is the data type of one feature. It has no requirements except compatibility with the distance metric.
 *
 * \c Distance is a functor that computes the distance between two VoctreeDescriptorT objects. It must have a \c result_type
 * typedef specifying the type of the returned distance. For the purposes of VocabularyTree, this need not even be
 * a metric; distances simply need to be comparable.
 *
 * \c VocDescAllocator is an STL-compatible allocator used to allocate Features internally.
 */
template<class VoctreeDescriptorT,
         template<typename, typename> class Distance = L2,
         class VocDescAllocator = typename DefaultAllocator<VoctreeDescriptorT>::type>
class VocabularyTree
{
public:
  /**
   * @brief Constructor, empty tree.
   *
   * @param d Functor for computing the distance between two features
   *
   * @todo Allocator parameter, also in MutableVocabularyTree, TreeBuilder...
   */
  VocabularyTree();

  /**
   * @brief Constructor, loads vocabulary from file.
   *
   * @param file Saved vocabulary file
   * @param d    Functor for computing the distance between two features
   */
  VocabularyTree(const std::string& file);
  
  /// Quantizes a feature into a discrete word.
  template<class OtherDescriptorT>
  Word quantize(const OtherDescriptorT& feature) const;
  
  /// Quantizes a feature into a discrete word.
  template<class OtherDescriptorT>
  Word onlySureFeaturesQuantize(const OtherDescriptorT& feature) const;

  template<class OtherDescriptorT>
  std::vector<Word> softQuantize(const OtherDescriptorT& feature, int n) const;
  
  template<class OtherDescriptorT>
  std::vector<Word> newSoftQuantize(const OtherDescriptorT& feature) const;
  
  /// Quantizes a set of features into sparse histogram of visual words.
  template<class OtherDescriptorT>
  SparseHistogram quantizeToSparse(const std::vector<OtherDescriptorT>& features) const;
  
  template<class DescriptorT>
  SparseHistogram softQuantizeToSparse(const std::vector<DescriptorT>& descriptors) const;
  
  //template<class DescriptorT>
  //SparseHistogram newSoftQuantizeToSparse(const std::vector<DescriptorT>& descriptors) const;

  /// Get the depth (number of levels) of the tree.
  uint32_t levels() const;
  /// Get the branching factor (max splits at each node) of the tree.
  uint32_t splits() const;
  /// Get the number of words the tree contains.
  uint32_t words() const;

  /// Clears vocabulary, leaving an empty tree.
  void clear();

  /// Save vocabulary to a file.
  void save(const std::string& file) const;
  /// Load vocabulary from a file.
  void load(const std::string& file);

  bool operator==(const VocabularyTree& other) const
  {
    if( (centers_ != other.centers_) ||
        (valid_centers_ != other.valid_centers_) ||
        (k_ != other.k_) ||
        (levels_ != other.levels_) ||
        (num_words_ != other.num_words_) ||
        (word_start_ != other.word_start_))
    {
      return false;
    }
    return true;
  }
protected:
  std::vector<VoctreeDescriptorT, VocDescAllocator> centers_;
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

template<class VoctreeDescriptorT, template<typename, typename> class Distance, class VocDescAllocator>
VocabularyTree<VoctreeDescriptorT, Distance, VocDescAllocator>::VocabularyTree()
: k_(0), levels_(0), num_words_(0), word_start_(0)
{
}

template<class VoctreeDescriptorT, template<typename, typename> class Distance, class VocDescAllocator>
VocabularyTree<VoctreeDescriptorT, Distance, VocDescAllocator>::VocabularyTree(const std::string& file)
: k_(0), levels_(0), num_words_(0), word_start_(0)
{
  load(file);
}

template<class VoctreeDescriptorT, template<typename, typename> class Distance, class VocDescAllocator>
template<class OtherDescriptorT>
Word VocabularyTree<VoctreeDescriptorT, Distance, VocDescAllocator>::quantize(const OtherDescriptorT& feature) const
{
  typedef typename Distance<VoctreeDescriptorT, OtherDescriptorT>::result_type distance_type;

  assert(initialized());
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
      distance_type child_distance = Distance<OtherDescriptorT, VoctreeDescriptorT>()(feature, centers_[child]);
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

template <typename distance_type> 
struct BestChild
{
  BestChild()
  {}
  
  BestChild(int32_t child, distance_type distance = std::numeric_limits<distance_type>::max())
  : child_(child)
  , distance_(distance)
  {}
  
  int32_t child_;
  distance_type distance_;
  
  bool operator<(const BestChild<distance_type>& otherChild) const
  { 
    return (distance_ < otherChild.distance_); 
  }
};

template<class VoctreeDescriptorT, template<typename, typename> class Distance, class VocDescAllocator>
template<class OtherDescriptorT>
std::vector<Word> VocabularyTree<VoctreeDescriptorT, Distance, VocDescAllocator>::softQuantize(const OtherDescriptorT& descriptor, int n) const
{
  typedef typename Distance<VoctreeDescriptorT, OtherDescriptorT>::result_type distance_type;

  assert(initialized());
  std::vector<BestChild<distance_type>> currentIndexes;
  std::vector<BestChild<distance_type>> futurIndexes;
  currentIndexes.push_back(-1);
  
  for(unsigned level = 0; level < levels_; ++level)
  {
    
    std::vector<BestChild<distance_type>> children;
    for(auto currentChild: currentIndexes)
    {
      // Calculate the offset to the first child of the current index.
      int32_t first_child = (currentChild.child_ + 1) * splits();
      // Find the child center closest to the query.
      for(int32_t child = first_child; child < first_child + (int32_t) splits(); ++child)
      {
        if(!valid_centers_[child])
          break; // Fewer than splits() children.
        distance_type child_distance = Distance<OtherDescriptorT, VoctreeDescriptorT>()(descriptor, centers_[child]);
        children.push_back(BestChild<distance_type>(child, child_distance));
      }
      
      std::partial_sort(children.begin(), children.begin() + n, children.end());
      children.resize(n);
      
      for(auto child: children)
      {
        futurIndexes.push_back(child);
      }
      children.clear();
    }
    currentIndexes.swap(futurIndexes);
    futurIndexes.clear();
  }
  std::sort(currentIndexes.begin(), currentIndexes.end());
  //currentIndexes.resize(n);
  std::vector<int32_t> res;

  for(BestChild<distance_type>& child: currentIndexes)
  {
    if(child.distance_ < 80000)
    {
      res.push_back(child.child_ - word_start_);
    }
  }
  return res;
}

/*template<class VoctreeDescriptorT, template<typename, typename> class Distance, class VocDescAllocator>
template<class OtherDescriptorT>
std::vector<Word> VocabularyTree<VoctreeDescriptorT, Distance, VocDescAllocator>::newSoftQuantize(const OtherDescriptorT& descriptor) const
{
  typedef typename Distance<VoctreeDescriptorT, OtherDescriptorT>::result_type distance_type;
  typedef typename VoctreeDescriptorT::value_type value_type;

  assert(initialized());
  std::vector<BestChild<distance_type>> currentIndexes;
  std::vector<BestChild<distance_type>> futurIndexes;
  std::vector<BestChild<distance_type>> otherIndexes;
  currentIndexes.push_back(-1);
  
  //A sortir de la fonction + valeur à changer
  float radius = 3.0;
  
  for(unsigned level = 0; level < levels_; ++level)
  {
    for(auto currentChild: currentIndexes)
    {
      // Calculate the offset to the first child of the current index.
      int32_t first_child = (currentChild.child_ + 1) * splits();
      
      // Find the children in the sphere
      BestChild<distance_type> closestChild;
      distance_type best_distance = std::numeric_limits<distance_type>::max();
      
      for(int32_t child = first_child; child < first_child + (int32_t) splits(); ++child)
      {
        if(!valid_centers_[child])
          break; // Fewer than splits() children.
        distance_type child_distance = Distance<OtherDescriptorT, VoctreeDescriptorT>()(descriptor, centers_[child]);
        
        //Child le plus proche
        if(child_distance < best_distance)
        {
          best_distance = child_distance;
          closestChild.child_ = child;
        }
        
        else {
          //Child dans la sphere
          if(child_distance < radius)
          {
            futurIndexes.push_back(BestChild<distance_type>(child, child_distance));
          }
          else
          {
            otherIndexes.push_back(BestChild<distance_type>(child, child_distance));
          }
        }
        
        futurIndexes.push_back(closestChild);
      }
      
      VoctreeDescriptorT bestDescriptor = centers_[closestChild.child_];

      //necessite une condition pour éviter de tester l'univers !
      for(BestChild<distance_type> child = otherIndexes.begin(); child != otherIndexes.end(); child++)
      {
        VoctreeDescriptorT secDescriptor = centers_[child];
        
        std::vector<value_type> projection(128);

        //projection du centre sur l'hyperplan
      
        //A REFAIRE
        
        distance_type d = Distance<VoctreeDescriptorT, std::vector<value_type>>()(bestDescriptor, projection);
        if(d < radius)
        {
          futurIndexes.push_back(BestChild<distance_type>(child, d));
        }
      }
      
    }

    currentIndexes.swap(futurIndexes);
    futurIndexes.clear();
  }
  
  std::sort(currentIndexes.begin(), currentIndexes.end());
  //currentIndexes.resize(n);
  std::vector<int32_t> res;

  for(BestChild<distance_type>& child: currentIndexes)
  {
    if(child.distance_ < 80000)
    {
      res.push_back(child.child_ - word_start_);
    }
  }
  return res;
}*/

template<class VoctreeDescriptorT, template<typename, typename> class Distance, class VocDescAllocator>
template<class OtherDescriptorT>
Word VocabularyTree<VoctreeDescriptorT, Distance, VocDescAllocator>::onlySureFeaturesQuantize(const OtherDescriptorT& feature) const
{
  typedef typename Distance<VoctreeDescriptorT, OtherDescriptorT>::result_type distance_type;
  typedef typename VoctreeDescriptorT::value_type value_type;
  float radius = sqrt(73000);

  assert(initialized());
  int32_t index = -1;
  
  for(unsigned level = 0; level < 1 ; ++level)
  {
    // Calculate the offset to the first child of the current index.
    int32_t first_child = (index + 1) * splits();
    
    // Find the child center closest to the query.
    int32_t best_child = first_child;
    distance_type best_distance = std::numeric_limits<distance_type>::max();
    
    for(int32_t child = first_child; child < first_child + (int32_t) splits(); ++child)
    {
      if(!valid_centers_[child])
        break;
      
      distance_type child_distance = sqrt(Distance<OtherDescriptorT, VoctreeDescriptorT>()(feature, centers_[child]));
      if(child_distance < best_distance)
      {
        best_child = child;
        best_distance = child_distance;
      }
    }
        
    index = best_child;
    
    VoctreeDescriptorT bestDescriptor = centers_[best_child];
    
    for(int32_t child = first_child; child < first_child + (int32_t) splits(); ++child)
    {
      if(child != best_child)
      {
        VoctreeDescriptorT secDescriptor = centers_[child];

        //A REFAIRE AVEC EIGEN + dimension à coder en propre
        std::vector<value_type> normal(128);
        std::vector<value_type> featureProj(128);
        float auxNormal = 0.0f;
        float normN = 0.0f;
        float d = 0.0f;
        
        //Normal Vector
        for(int i = 0; i < 128; ++i)
        {
          normal[i] = secDescriptor[i] - bestDescriptor[i];
          auxNormal += normal[i]*normal[i];
        }
        
        //Normalization:
        normN = sqrt(auxNormal);
        for(int i = 0; i < 128; ++i)
        {
          d += (bestDescriptor[i] + normal[i]/2) * normal[i] / normN;
          normal[i] = normal[i] /  normN;
        }
        
        //projection du centre sur l'hyperplan:
        
        //Produit scalaire feature.normal
        float scalarPdtFeatNorm = 0.0f;
        for(int i = 0; i < 128; ++i)
        {
          scalarPdtFeatNorm += feature[i]*normal[i];
        }
        
        //calcul projection
        for(int i = 0; i < 128; ++i)
        {
          featureProj[i] = feature[i] - (scalarPdtFeatNorm - d) * normal[i];
        }
        
        float r = abs((scalarPdtFeatNorm - d));
        if(r < radius)
        {
          //distance_type dist = sqrt(Distance<VoctreeDescriptorT, std::vector<value_type>>()(bestDescriptor, featureProj));
          //for(int32_t childToCompare = first_child; childToCompare < first_child + (int32_t) splits(); ++childToCompare)
          //{
          //  distance_type distToCompare = sqrt(Distance<VoctreeDescriptorT, std::vector<value_type>>()(centers_[childToCompare], featureProj));
          //  if(distToCompare < dist + 0.001)
          //  {
              return -1;
          //  }       
          //}
        }
        
      }
      
    }
  }

  return index - word_start_;
}

template<class VoctreeDescriptorT, template<typename, typename> class Distance, class VocDescAllocator>
template<class DescriptorT>
SparseHistogram VocabularyTree<VoctreeDescriptorT, Distance, VocDescAllocator>::quantizeToSparse(const std::vector<DescriptorT>& descriptors) const
{
  SparseHistogram histo;
  std::vector<Word> doc(descriptors.size(), 0);
  int sizeDesc = descriptors.size();
  int nbDescriptors = 0;

  //#pragma omp parallel for
  for(size_t j = 0; j < descriptors.size(); ++j)
  { 
    //doc[j] = quantize(descriptors[j]);
    if(onlySureFeaturesQuantize(descriptors[j]) != -1)
     {
      doc[j] = onlySureFeaturesQuantize(descriptors[j]);
      nbDescriptors++;
    }
  }
  std::cout << nbDescriptors << " descripteurs sur " << sizeDesc << " sont sans ambiguité" << std::endl;
  computeSparseHistogram(doc, histo);
  return histo;
}

template<class VoctreeDescriptorT, template<typename, typename> class Distance, class VocDescAllocator>
template<class DescriptorT>
SparseHistogram VocabularyTree<VoctreeDescriptorT, Distance, VocDescAllocator>::softQuantizeToSparse(const std::vector<DescriptorT>& descriptors) const
{
  SparseHistogram histo;
  
//  std::size_t descSize = std::min(descriptors.size(), (std::size_t)500);
  std::size_t descSize = descriptors.size();
  // quantize the features
  #pragma omp parallel for
  for(size_t j = 0; j < descSize; ++j)
  {
    // store the visual word associated to the feature in the temporary list
    std::vector<Word> quantizedDescriptors = softQuantize<DescriptorT>(descriptors[j], 2);
    #pragma omp critical
    {
      for(auto qDesc: quantizedDescriptors)
      {
        histo[qDesc].push_back(j);
      }
    }
  }
  return histo;
}

template<class VoctreeDescriptorT, template<typename, typename> class Distance, class VocDescAllocator>
uint32_t VocabularyTree<VoctreeDescriptorT, Distance, VocDescAllocator>::levels() const
{
  return levels_;
}

template<class VoctreeDescriptorT, template<typename, typename> class Distance, class VocDescAllocator>
uint32_t VocabularyTree<VoctreeDescriptorT, Distance, VocDescAllocator>::splits() const
{
  return k_;
}

template<class VoctreeDescriptorT, template<typename, typename> class Distance, class VocDescAllocator>
uint32_t VocabularyTree<VoctreeDescriptorT, Distance, VocDescAllocator>::words() const
{
  return num_words_;
}

template<class VoctreeDescriptorT, template<typename, typename> class Distance, class VocDescAllocator>
void VocabularyTree<VoctreeDescriptorT, Distance, VocDescAllocator>::clear()
{
  centers_.clear();
  valid_centers_.clear();
  k_ = levels_ = num_words_ = word_start_ = 0;
}

template<class VoctreeDescriptorT, template<typename, typename> class Distance, class VocDescAllocator>
void VocabularyTree<VoctreeDescriptorT, Distance, VocDescAllocator>::save(const std::string& file) const
{
  /// @todo Support serializing of non-"simple" feature classes
  /// @todo Some identifying name for the distance used
  assert(initialized());

  std::ofstream out(file.c_str(), std::ios_base::binary);
  out.write((char*) (&k_), sizeof (uint32_t));
  out.write((char*) (&levels_), sizeof (uint32_t));
  uint32_t size = centers_.size();
  out.write((char*) (&size), sizeof (uint32_t));
  out.write((char*) (&centers_[0]), centers_.size() * sizeof (VoctreeDescriptorT));
  out.write((char*) (&valid_centers_[0]), valid_centers_.size());
}

template<class VoctreeDescriptorT, template<typename, typename> class Distance, class VocDescAllocator>
void VocabularyTree<VoctreeDescriptorT, Distance, VocDescAllocator>::load(const std::string& file)
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
    in.read((char*) (&centers_[0]), centers_.size() * sizeof (VoctreeDescriptorT));
    in.read((char*) (&valid_centers_[0]), valid_centers_.size());
  }
  catch(std::ifstream::failure& e)
  {
    throw std::runtime_error("Failed to load vocabulary tree file" + file);
  }

  setNodeCounts();
  assert(size == num_words_ + word_start_);
}

template<class VoctreeDescriptorT, template<typename, typename> class Distance, class VocDescAllocator>
void VocabularyTree<VoctreeDescriptorT, Distance, VocDescAllocator>::setNodeCounts()
{
  num_words_ = k_;
  word_start_ = 0;
  for(uint32_t i = 0; i < levels_ - 1; ++i)
  {
    word_start_ += num_words_;
    num_words_ *= k_;
  }
}

}
}
#endif //OPENMVG_VOCABULARY_TREE_VOCABULARY_TREE_HPP
