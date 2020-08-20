// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Database.hpp"
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/tail.hpp>
#include <boost/progress.hpp>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <boost/format.hpp>

namespace aliceVision{
namespace voctree{

std::ostream& operator<<(std::ostream& os, const SparseHistogram &dv)	
{
	for( const auto &e : dv )
	{
		os << e.first << ", " << e.second.size() << "; ";
	}
	os << "\n";
	return os;
}

Database::Database(uint32_t num_words)
: word_files_(num_words),
word_weights_( num_words, 1.0f ) { }

DocId Database::insert(DocId doc_id, const SparseHistogram& document)
{
  // Ensure that the new document to insert is not already there.
  assert(database_.find(doc_id) == database_.end());

  // For each word, retrieve its inverted file and increment the count for doc_id.
  for(SparseHistogram::const_iterator it = document.begin(), end = document.end(); it != end; ++it)
  {
    Word word = it->first;
    InvertedFile& file = word_files_[word];
    if(file.empty() || file.back().id != doc_id)
      file.push_back(WordFrequency(doc_id, it->second.size()));
    else
      file.back().count += it->second.size();
  }

  database_[doc_id] = document;

  return doc_id;
}

void Database::sanityCheck(std::size_t N, std::map<std::size_t, DocMatches>& matches) const
{
  // if N is equal to zero
  if(N == 0)
  {
    // retrieve all the matchings
    N = this->size();
  }
  else
  {
    // otherwise always take the min between N and the number of documents
    // in the database
    N = std::min(N, this->size());
  }

  matches.clear();
  // since we already know the size of the vectors, in order to parallelize the 
  // query allocate the whole memory
  boost::progress_display display(database_.size());
  
  //#pragma omp parallel for default(none) shared(database_)
  for(const auto &doc : database_)
  {
    std::vector<DocMatch> m;
    find(doc.second, N, m);
    //		matches.emplace_back( m );
    matches[doc.first] = m;
    ++display;
  }
}

/**
 * @brief Find the top N matches in the database for the query document.
 *
 * @param[in]  document The query document, a set of quantized words.
 * @param[in]  N        The number of matches to return.
 * @param[out] matches  IDs and scores for the top N matching database documents.
 * @param[in] distanceMethod the method used to compute distance between histograms.
 */
void Database::find(const std::vector<Word>& document, std::size_t N, std::vector<DocMatch>& matches, const std::string &distanceMethod) const
{
  SparseHistogram query;
  // from the list of visual words associated with each feature in the document/image
  // generate the (sparse) histogram of the visual words 
  computeSparseHistogram(document, query);

  find( query, N, matches, distanceMethod);
}

/**
 * @brief Find the top N matches in the database for the query document.
 *
 * @param      query The query document, a normalized set of quantized words.
 * @param      N        The number of matches to return.
 * @param[out] matches  IDs and scores for the top N matching database documents.
 * @param[in] distanceMethod the method used to compute distance between histograms.
 */
void Database::find( const SparseHistogram& query, std::size_t N, std::vector<DocMatch>& matches, const std::string &distanceMethod) const
{
    matches.clear();
    matches.reserve(database_.size());
    for(const auto& document : database_)
    {
        // for each document/image in the database compute the distance between the
        // histograms of the query image and the others
        const float distance = sparseDistance(query, document.second, distanceMethod, word_weights_);
        matches.emplace_back(document.first, distance);
    }
    const std::size_t nMatches = std::min(N, matches.size());
    std::partial_sort(matches.begin(), matches.begin() + nMatches, matches.end());
    matches.resize(nMatches);
}

/**
 * @brief Compute the TF-IDF weights of all the words. To be called after inserting a corpus of
 * training examples into the database.
 *
 * @param default_weight The default weight of a word that appears in none of the training documents.
 */
void Database::computeTfIdfWeights(float default_weight)
{
  float N = (float) database_.size();
  std::size_t num_words = word_files_.size();
  for(std::size_t i = 0; i < num_words; ++i)
  {
    std::size_t Ni = word_files_[i].size();
    if(Ni != 0)
      word_weights_[i] = std::log(N / Ni);
    else
      word_weights_[i] = default_weight;
  }
}

void Database::saveWeights(const std::string& file) const
{
  std::ofstream out(file.c_str(), std::ios_base::binary);
  uint32_t num_words = word_weights_.size();
  out.write((char*) (&num_words), sizeof (uint32_t));
  out.write((char*) (&word_weights_[0]), num_words * sizeof (float));
}

void Database::loadWeights(const std::string& file)
{
  std::ifstream in;
  in.exceptions(std::ifstream::eofbit | std::ifstream::failbit | std::ifstream::badbit);

  try
  {
    in.open(file.c_str(), std::ios_base::binary);
    uint32_t num_words = 0;
    in.read((char*) (&num_words), sizeof (uint32_t));
    word_files_.resize(num_words); // Inverted files start out empty
    word_weights_.resize(num_words);
    in.read((char*) (&word_weights_[0]), num_words * sizeof (float));
  }
  catch(std::ifstream::failure& e)
  {
    throw std::runtime_error((boost::format("Failed to load vocabulary weights file '%s'") % file).str());
  }
}

///**
// * Normalize a document vector representing the histogram of visual words for a given image
// * 
// * @param[in/out] v the unnormalized histogram of visual words
// */
//void Database::normalize(SparseHistogram& v) const
//{
//  float sum = 0.0f;
//  for(SparseHistogram::iterator i = v.begin(), ie = v.end(); i != ie; ++i)
//    sum += i->second;
//  float inv_sum = 1.0f / sum;
//  for(SparseHistogram::iterator i = v.begin(), ie = v.end(); i != ie; ++i)
//    i->second *= inv_sum;
//}

/**
 * @brief Return the size of the database in terms of number of documents
 * @return the number of documents
 */
std::size_t Database::size() const
{
  return database_.size();
}

} //namespace voctree
} //namespace aliceVision
