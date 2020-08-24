// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "VocabularyTree.hpp"
#include <aliceVision/types.hpp>

#include <map>
#include <cstddef>
#include <string>

namespace aliceVision{
namespace voctree{

/**
 * @brief Struct representing a single database match.
 *
 * \c score is in the range [0,2], where 0 is best and 2 is worst.
 */
struct DocMatch
{
  DocId id{UndefinedIndexT};
  float score{0.0f};

  DocMatch() = default;
  DocMatch(DocId _id, float _score)
    : id(_id)
    , score(_score)
  {}

  /// Allows sorting DocMatches in best-to-worst order with std::sort.
  bool operator<(const DocMatch& other) const
  {
    return score < other.score;
  }

  bool operator==(const DocMatch& other) const
  {
    return id == other.id &&
           score == other.score;
  }
  bool operator!=(const DocMatch& other) const
  {
    return !(*this == other);
  }
};

typedef std::vector<DocMatch> DocMatches;

/**
 * @brief Class for efficiently matching a bag-of-words representation of a document (image) against
 * a database of known documents.
 */
class Database
{
public:
  /**
   * @brief Constructor
   *
   * If computing weights for a new vocabulary, \c num_words should be the size of the vocabulary.
   * If calling loadWeights(), it can be left zero.
   */
  explicit Database(uint32_t num_words = 0);

  /**
   * @brief Insert a new document.
   *
   * @param doc_id Unique ID of the new document to insert
   * @param document The set of quantized words in a document/image.
   * \return An ID representing the inserted document.
   */
  DocId insert(DocId doc_id, const SparseHistogram& document);

  /**
   * @brief Perform a sanity check of the database by querying each document
   * of the database and finding its top N matches
   * 
   * @param[in] N The number of matches to return.
   * @param[out] matches IDs and scores for the top N matching database documents.
   */
   void sanityCheck(std::size_t N, std::map<std::size_t, DocMatches>& matches) const;

  /**
   * @brief Find the top N matches in the database for the query document.
   *
   * @param[in] document The query document, a set of quantized words.
   * @param[in] N        The number of matches to return.
   * @param[in] distanceMethod distance method (norm L1, etc.)
   * @param[out] matches  IDs and scores for the top N matching database documents.
   */
  void find(const std::vector<Word>& document, std::size_t N, std::vector<DocMatch>& matches, const std::string &distanceMethod = "strongCommonPoints") const;
  
    /**
   * @brief Find the top N matches in the database for the query document.
   *
   * @param[in] query The query document, a normalized set of quantized words.
   * @param[int] N        The number of matches to return.
   * @param[in] distanceMethod distance method (norm L1, etc.)
   * @param[out] matches  IDs and scores for the top N matching database documents.
   */
  void find(const SparseHistogram& query, std::size_t N, std::vector<DocMatch>& matches, const std::string &distanceMethod = "strongCommonPoints") const;

  /**
   * @brief Compute the TF-IDF weights of all the words. To be called after inserting a corpus of
   * training examples into the database.
   *
   * @param default_weight The default weight of a word that appears in none of the training documents.
   */
  void computeTfIdfWeights(float default_weight = 1.0f);

  /**
   * @brief Return the size of the database in terms of number of documents
   * @return the number of documents
   */
  std::size_t size() const;

  /// Save the vocabulary word weights to a file.
  void saveWeights(const std::string& file) const;
  /// Load the vocabulary word weights from a file.
  void loadWeights(const std::string& file);

  // Save weights and documents
  //void save(const std::string& file) const;
  //void load(const std::string& file);

  const SparseHistogramPerImage& getSparseHistogramPerImage() const
  {
    return database_;
  }
  
private:

  struct WordFrequency
  {
    DocId id;
    uint32_t count;

    WordFrequency() = default;
    WordFrequency(DocId _id, uint32_t _count)
      : id(_id)
      , count(_count)
    {}
  };

  // Stored in increasing order by DocId
  typedef std::vector<WordFrequency> InvertedFile;

  /// @todo Use sorted vector?
  // typedef std::vector< std::pair<Word, float> > DocumentVector;
  
  friend std::ostream& operator<<(std::ostream& os, const SparseHistogram& dv);

  std::vector<InvertedFile> word_files_;
  std::vector<float> word_weights_;
  SparseHistogramPerImage database_; // Precomputed for inserted documents

  /**
   * Normalize a document vector representing the histogram of visual words for a given image
   * @param[in/out] v the unnormalized histogram of visual words
   */
  void normalize(SparseHistogram& v) const;
};

}//namespace voctree
}//namespace aliceVision
