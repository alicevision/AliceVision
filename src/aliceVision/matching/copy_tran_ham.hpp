// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_MATCHING_ARRAYMATCHER_BRUTE_FORCE_H
#define OPENMVG_MATCHING_ARRAYMATCHER_BRUTE_FORCE_H

#include "openMVG/numeric/numeric.h"
#include "openMVG/matching/matching_interface.hpp"
#include "openMVG/matching/metric.hpp"
#include "openMVG/stl/indexed_sort.hpp"
#include <openMVG/config.hpp>
#include <memory>
#include <iostream>
#include <string.h>
#include <stdlib.h>
#include <unordered_map>

namespace openMVG {
namespace matching {

// By default compute square(L2 distance).
template < typename Scalar = float, typename Metric = L2_Simple<Scalar> >
class ArrayMatcherBruteForce  : public ArrayMatcher<Scalar, Metric>
{
  public:
  typedef typename Metric::ResultType DistanceType;

  ArrayMatcherBruteForce()   {}
  virtual ~ArrayMatcherBruteForce() {
    memMapping.reset();
  }

  /**
   * Build the matching structure
   *
   * \param[in] dataset   Input data.
   * \param[in] nbRows    The number of component.
   * \param[in] dimension Length of the data contained in the dataset.
   *
   * \return True if success.
   */
  bool Build(const Scalar * dataset, int nbRows, int dimension) {
    if (nbRows < 1) {
      memMapping.reset(nullptr);
      return false;
    }
    memMapping.reset(new Eigen::Map<BaseMat>( (Scalar*)dataset, nbRows, dimension) );
    return true;
  }

  /**
   * Search the nearest Neighbor of the scalar array query.
   *
   * \param[in]   query     The query array
   * \param[out]  indice    The indice of array in the dataset that
   *  have been computed as the nearest array.
   * \param[out]  distance  The distance between the two arrays.
   *
   * \return True if success.
   */
  bool SearchNeighbour( const Scalar * query,
                        int * indice, DistanceType * distance)
  {
    if (memMapping.get() == nullptr)
      return false;

      //matrix representation of the input data;
      Eigen::Map<BaseMat> mat_query((Scalar*)query, 1, (*memMapping).cols() );
      Metric metric;
      std::vector<DistanceType> vec_dist((*memMapping).rows(), 0.0);
    for (int i = 0; i < (*memMapping).rows(); ++i)
    {
        // Compute Distance Metric
        vec_dist[i] = metric( (Scalar*)query, (*memMapping).row(i).data(), (*memMapping).cols() );
      }
      if (!vec_dist.empty())
      {
        // Find the minimum distance :
        typename std::vector<DistanceType>::const_iterator min_iter =
          min_element( vec_dist.begin(), vec_dist.end());
        *indice =std::distance(
          typename std::vector<DistanceType>::const_iterator(vec_dist.begin()),
          min_iter);
        *distance = static_cast<DistanceType>(*min_iter);
      }
      return true;
  }


    //Global
    //struct descriptor desc[(*memMapping).rows()]; virka ikke finner ikke memapping 
    
struct descriptor
{
    int change;
    int idx;
    unsigned char *data;
};

struct area
{
    int begin;
    int end;
    std::unordered_map<std::string, struct area> *pool = NULL;
};

    
/* hackers delight*/
void transpose8rS64( unsigned char* A, int m, int n, unsigned char* B ) 
{
	unsigned long long x, t;
	int i;

	for ( i = 0; i <= 7; i++ )     // Load 8 bytes from the
		x = x << 8 | A[m*i];      // input array and pack
								  // them into x.

	t = (x ^ (x >> 7)) & 0x00AA00AA00AA00AALL;
	x = x ^ t ^ (t << 7);
	t = (x ^ (x >> 14)) & 0x0000CCCC0000CCCCLL;
	x = x ^ t ^ (t << 14);
	t = (x ^ (x >> 28)) & 0x00000000F0F0F0F0LL;
	x = x ^ t ^ (t << 28);

	for ( i = 7; i >= 0; i-- ) 
	{   // Store result into
		B[n*i] = x; x = x >> 8;
	}  // output array B.
}


void transpose( unsigned char *A, unsigned char *B ) 
{
  int i = 0;
  for ( i = 0; i < 8 * 16; i += 8 ) 
		transpose8rS64( A + i, 1, 1, B + i );
}


int hamming_distance( unsigned char* A, unsigned char* B ) 
{
	unsigned int g[4];
	int sum = 0;

    g[0] = *(unsigned int *)A ^ *(unsigned int *)B;
    g[1] = *(unsigned int *)(A + 4) ^ *(unsigned int *)(B + 4);
    g[2] = *(unsigned int *)(A + 8) ^ *(unsigned int *)(B + 8);
    g[3] = *(unsigned int *)(A + 12) ^ *(unsigned int *)(B + 12);

    sum += __builtin_popcount(*g);
    sum += __builtin_popcount(*(g + 1));
    sum += __builtin_popcount(*(g + 2));
    sum += __builtin_popcount(*(g + 3));
    
    return sum;
}



void organize( unsigned char* A, unsigned char* B )
{
  int i;
  int j = 0;
  int cnt = 0;
  unsigned char tmp;
  for (int j = 0; j < 8; j++)
  for ( i = 0; i < 8 * 16; i += 8 ) {
	B[cnt] = A[i + j];
	cnt++;
	
      }
}



static int
cmp(const void *p1, const void *p2)
{

    unsigned char * ptr1 = ((struct descriptor *)p1)->data;
    unsigned char * ptr2 = ((struct descriptor *)p2)->data;
    
    int i = 0;
    int tmp = *ptr1 - *ptr2;
    while (tmp == 0) {
        ptr1++;
        ptr2++;
        tmp = *ptr1 - *ptr2;
        
        if (i == 127) return 0;
        i++;
    }
    
    return tmp;
}

    static int
string_cmp(const void *p1, const void *p2)
{

    unsigned char * ptr1 = ( unsigned char * )p1;
    unsigned char * ptr2 = ( unsigned char * )p2;
    
    int i = 0;
    int tmp = *ptr1 - *ptr2;
    while (tmp == 0) {
        ptr1++;
        ptr2++;
        tmp = *ptr1 - *ptr2;
        
        if (i == 127) return 0;
        i++;
        
    }
    
    return tmp;
}
    

    int compareTo(unsigned char * A, unsigned char * B )
    {
        for (int i = 0; i < 4; i++)
        {
            int tmp = string_cmp( A+i*128, B+i*128 );
            if (tmp != 0) return tmp;
        }
        
  

        return 0;
    }



    //transpose & set up
    void transpose_and_sort(unsigned char *transposed_array, struct descriptor *desc, const int ROWS, const int COLS)
    {
        Scalar tmp_row[COLS]; 
        unsigned char A[128];
        unsigned char B[128]; 
        
        for (int i = 0; i < ROWS; ++i)
        {
            Scalar * test_row = (*memMapping).data();
            memcpy(tmp_row, test_row+(i*COLS), COLS);
            transpose( (unsigned char *)tmp_row, A );                
            organize( A, B );
            
            memcpy( transposed_array + (i*COLS), B, 128 );
            desc[i].idx = i;
            desc[i].data = transposed_array + (i*COLS);
        }

        //Sort the array
        qsort(desc, ROWS, sizeof(struct descriptor), cmp);
    }


    /*void build_pool(std::unordered_map<std::string, struct area> pool, struct descriptor *desc, const int ROWS) {

        
        for (int i = 0; i < ROWS; i++)
        {
            std::string s1( (char *)desc[i].data, 16 );
            std::string s2( (char *)desc[i].data+16, 16 );

            
            //depth[10] "s1, s2, s3, s4";
            //ACE sample has no need for more than depth 2
            //EMP needs more
            //How to determid depth??? Could check size of pool or number of pools in given depth
            //number of pools could be bad as there could be one large and several small ones
        
            if (pool.find(s1) != pool.end()) //if s1 exists in pool increase area
                pool[s1].end = i;
            else //else create new entry
            {
                struct area b;
                std::unordered_map<std::string, struct area> *tmp_pool = new std::unordered_map<std::string, struct area>; 
                b.begin = i;
                b.end = i;
                b.pool =  tmp_pool;
                pool[s1] = b;
            }
        
        
            if (pool[s1].pool->find(s2) != pool[s1].pool->end())  //if s2 exists in pool[s1].pool increase area
            {
                std::unordered_map<std::string, struct area> &t_pool = *pool[s1].pool;
                t_pool[s2].end = i;
            }
            else  //else create new entry
            {
                struct area b;
                std::unordered_map<std::string, struct area> *tmp_pool = new std::unordered_map<std::string, struct area>;
                b.begin = i;
                b.end = i;
                b.pool =  tmp_pool;
                std::unordered_map<std::string, struct area> &t_pool = *pool[s1].pool;
                t_pool[s2] = b;
            }    
        }
    }*/
    /**
     * Search the N nearest Neighbor of the scalar array query.
   *
   * \param[in]   query     The query array
   * \param[in]   nbQuery   The number of query rows
   * \param[out]  indices   The corresponding (query, neighbor) indices
   * \param[out]  distances The distances between the matched arrays.
   * \param[out]  NN        The number of maximal neighbor that will be searched.
   *
   * \return True if success.
   */
  bool SearchNeighbours
  (
    const Scalar * query, int nbQuery,
    IndMatches * pvec_indices,
    std::vector<DistanceType> * pvec_distances,
    size_t NN
  )
  {
    if (memMapping.get() == nullptr)  {
      return false;
    }

    if (NN > (*memMapping).rows() || nbQuery < 1) {
      return false;
    }

    //matrix representation of the input data;
    Eigen::Map<BaseMat> mat_query((Scalar*)query, nbQuery, (*memMapping).cols());
    Metric metric;

    pvec_distances->resize(nbQuery * NN);
    pvec_indices->resize(nbQuery * NN);

    int i;
    unsigned char A[128];
    unsigned char B[128]; 
    const int COLS = (*memMapping).cols();
    const int ROWS = (*memMapping).rows();
    std::unordered_map<std::string, struct area> pool;
    struct descriptor desc[ROWS];
    unsigned char *transposed_array = (unsigned char * )malloc(ROWS * COLS); //remember to free memory

    int hamming_counter = 0;
    int pool_counter = 0;

    //transpose & set up
    transpose_and_sort(transposed_array, desc, ROWS, COLS);
    //build_pool(pool, desc, ROWS);
    //build the pool tree
     for (i = 0; i < ROWS; i++)
    {
        std::string s1( (char *)desc[i].data, 16 );
        std::string s2( (char *)desc[i].data+16, 16 );
        
        //std::string s3( (char *)desc[i].data+32, 16 );
        //std::string s4( (char *)desc[i].data+48, 16 );

        //depth[10] "s1, s2, s3, s4";
        //ACE sample has no need for more than depth 2
        //EMP needs more
        //How to determid depth??? Could check size of pool or number of pools in given depth
        //number of pools could be bad as there could be one large and several small ones
        
        if (pool.find(s1) != pool.end()) //if s1 exists in pool increase area
            pool[s1].end = i;
        else //else create new entry
        {
            struct area b;
            std::unordered_map<std::string, struct area> *tmp_pool = new std::unordered_map<std::string, struct area>; 
            b.begin = i;
            b.end = i;
            b.pool =  tmp_pool;
            pool[s1] = b;
        }
        
        
        if (pool[s1].pool->find(s2) != pool[s1].pool->end())  //if s2 exists in pool[s1].pool increase area
        {
            std::unordered_map<std::string, struct area> &t_pool = *pool[s1].pool;
            t_pool[s2].end = i;
        }
        else  //else create new entry
        {
            struct area b;
            std::unordered_map<std::string, struct area> *tmp_pool = new std::unordered_map<std::string, struct area>;
            b.begin = i;
            b.end = i;
            b.pool =  tmp_pool;
            std::unordered_map<std::string, struct area> &t_pool = *pool[s1].pool;
            t_pool[s2] = b;
        }    
        }
    /*
     Some usefull prints to look at the blocks.
    ********************************************************
    std::cout << "Pool size: " << pool.size() << std::endl;

    std::unordered_map<std::string, struct area> &t_pool = *pool[s1].pool;
    for(auto iter = pool.begin(); iter != pool.end(); ++iter){
        auto cur = iter->first; // pointer to Node
        std::cout << std::endl;
        std::cout << "begin: " << pool[cur].begin << " end: " << pool[cur].end << std::endl;

        std::unordered_map<std::string, struct area> &t_pool = *pool[cur].pool;

        std::cout << "2_Pool size: " << t_pool.size() << std::endl;
        for(auto iter2 = t_pool.begin(); iter2 != t_pool.end(); ++iter2){
            auto cur2 = iter2->first; // pointer to Node
            
            std::cout << "tbegin: " << t_pool[cur2].begin << " end: " << t_pool[cur2].end << std::endl;
        
        }
    }
    ********************************************************
    */

    
    int cnt = 0;
    int firstCounter = 0;
    int lastCounter = 0;
    for (int i=0; i < nbQuery; ++i)
    {
        const Scalar * queryPtr = mat_query.row(i).data();
        transpose( (unsigned char *)queryPtr, A );
        organize( A, B );

        std::string s1( (char *)B, 16 ) ;
        std::string s2( (char *)B + 16, 16 ) ; //fixed offset

        int pos;
        int begin = 0, end = ROWS; //Need eception if no pool excist, we search the complete area in this case for now
        bool found_match_level1 = true; //found match in most significant bit
        bool found_match_level2 = false; //found match in second most significant bit
        
        if (pool.find(s1) != pool.end()) //if s1 exists in pool
        {
            if (pool[s1].pool->find(s2) != pool[s1].pool->end()) //if s2 exists in pool[s1].pool
            {

                //printf("EXACT MATCH\n");
                std::unordered_map<std::string, struct area> &t_pool = *pool[s1].pool;
                begin = t_pool[s2].begin;
                end = t_pool[s2].end;
                cnt++;
                found_match_level2 = true;
            }
            else
            {
                //printf("NO\n");
                begin = pool[s1].begin;
                end = pool[s1].end;
            } 
        } else {
            found_match_level1 = false;
        }
        
        int diff0, diff1, diff2, diff3, dota;
        int best = -1, nextBest = -1;
        int tmp, res;
        int bestIndex = -1;
        int nextBestIndex = -1;

        
        //std::cout << "end-begin" << end-begin << " begin: " << begin << " end: " << end << std::endl;
        //if(end-begin > 100)
        if (end-begin != -8) //the search area is to small, try hamming distance instead to find the desired index
            {
                //printf("ORIGINAL SEARCH AREA: %d-%d\n", begin, end);
                //   if (begin != 0) begin--;
                // else end++;
                hamming_counter++;

                bool level0 = false; //all descriptors 
                bool level1 = false; //descriptors from most sigificant bit
                bool level2 = false; //desctiptors from second most significant bit
                
                bool first = false;
                int size = 0;
                //Check if we have two descriptors inside this pool.
                //This is likely to be false in some cases as most but not
                //all descriptors have all zero's in their most significant bit.

                
                if(found_match_level2) {
                    if(end-begin < 1) { //is level2 too small?                    
                        if(pool[s1].end-pool[s1].begin < 1) { //is level1 too small?
                            level0 = true;
                            size = pool.size();
                        } else {
                            //first = true;
                            level1 = true;
                            size = pool[s1].pool->size();   
                        }
                    } else {
                        level2 = true;
                        std::unordered_map<std::string, struct area> &t_pool = *pool[s1].pool;
                        size = t_pool[s2].pool->size();
                    }                    
                } else if(found_match_level1) {
                    if(end-begin < 1) {
                        level0 = true;
                        size = pool.size();
                    } else {
                        level1 = true;
                        size = pool[s1].pool->size();                           
                    }
                } else {
                    level0 = true;
                    size = pool.size();
                }

                
                //size = pool[s1].pool->size();
                /*
                if(pool[s1].end-pool[s1].begin < 2) {
                    first = true;
                    size = pool.size();
                }
                */
                unsigned char bestPool[size*16];
                unsigned char nBestPool[size*16];


                int bCount = 0;
                int nbCount = 0;


                int bestHam = 130;
                int nBestHam = 130;


                //else if(pool[s1].pool[s2].end - pool[s1].pool[s2].begin < 2)

            
                //Sort the patterns by hamming distance, in progress
                //****************************************************''

                //We check the hammin distance between the descriptor we are looking for
                //and all the descriptors that have not been filtered out by the first
                //hamming match check. If (first) is true then we did not get a match in the
                //most significant bit and we will instead match using hamming distance
                //between all possible pools. 

                if(level0) {
firstCounter++;
                    int min[pool.size()];
                    unsigned char min_pattern[pool.size() * 16];
                    int hamming;
                    //printf("%d\n", pool.size());
                    //for (int h = 0; h < pool.size(); h++)
                    //  {
                    //printf("bb[%d]\tee[%d]\n", pool[s1].pool->begin(), pool[s1].pool->end());

                    //We initialize hamming distance and iterate through the current pool and find the minimum distance.
                    hamming = 130;

                    
                    for(auto iter = pool.begin(); iter != pool.end(); ++iter)
                        {
                            auto c = iter->first;
                            unsigned char *cur = (unsigned char *)&c[0];
                            unsigned char *str = (unsigned char *)&s1[0];
                            
                            hamming = hamming_distance( str, cur );
                            
                            if(hamming <= bestHam) {
                                if(hamming == bestHam) { //add indexes to current best hamming distance
                                    memcpy(bestPool+ (bCount*16), cur, 16);
                                    bCount++;

                                } else { //new best hamming distance, add new index
                                    memcpy(nBestPool, bestPool, bCount*16);
                                    nbCount = bCount;
                                    bCount = 0;
                                    nBestHam = bestHam;
                                    bestHam = hamming;
                                    memcpy(bestPool + bCount*16, cur, 16);   
                                    bCount++;
                                }
                            } else if(hamming <= nBestHam) {
                                if(hamming == nBestHam) { //add indexes to next best hamming distance
                                    memcpy(nBestPool+(nbCount*16), cur, 16);
                                    nbCount++;
                                } else { //new next best hamming, add new index                                                                    
                                    nbCount = 0;
                                    memcpy(nBestPool, cur, 16);
                                    nbCount++;
                                    nBestHam = hamming;
                                }
                            }                                                                                                            
                        }                          
                }

                if(level1) {
                    lastCounter++;
                    int min[pool[s1].pool->size()];                  
                    unsigned char min_pattern[pool[s1].pool->size() * 16];
                    int hamming;


                    //for (int h = 0; h < pool[s1].pool->size(); h++)
                    //   {
                    //printf("b[%d]\te[%d]\n", pool[s1].pool->begin(), pool[s1].pool->end());
                    hamming = 130;
                    //printf("!!!!!!!!!!!!!!%d\n", pool[s1].pool->size());
                    for(auto iter = pool[s1].pool->begin(); iter != pool[s1].pool->end(); ++iter)
                        {                                   
                            auto c = iter->first;
                            unsigned char *cur = (unsigned char *)&c[0];
                            unsigned char *str = (unsigned char *)&s2[0];

                            //std::unordered_map<std::string, struct area> &t_pool = *pool[s1].pool;
                            //printf("%d\t%d\n", t_pool[c].begin, t_pool[c].end);
                            hamming = hamming_distance( str, cur );
                                                                       
                            if(hamming <= bestHam) {
                                if(hamming == bestHam) {
                                    memcpy(bestPool+(bCount*16), cur, 16);
                                    bCount++;
                                } else {
                                    memcpy(nBestPool, bestPool, 16); //might be wrong
                                    //memcpy(nBestPool, bestPool, bCount*16); // is the size paramater here correct? PROBABLY memcpy(,,16)
                                    nbCount = bCount;
                                    bCount = 0;
                                    nBestHam = bestHam;
                                    bestHam = hamming;
                                    memcpy(bestPool + bCount*16, cur, 16);   
                                    bCount++;
                                }
                            } else if(hamming <= nBestHam) {
                                if(hamming == nBestHam) {
                                    memcpy(nBestPool+(nbCount*16), cur, 16);
                                    nbCount++;
                                } else {                                           
                                    nbCount = 0;
                                    memcpy(nBestPool, cur, 16);
                                    nbCount++;
                                    nBestHam = hamming;
                                }
                            }                                                                                                             
                        }
                    //   }
                            
                
                }

                if(level2) {                    
                    bCount = 1;
                    nbCount = 0;
                }
                    
                /*
                  if(first) {
                  firstCounter++;
                  int min[pool.size()];
                  unsigned char min_pattern[pool.size() * 16];
                    int hamming;
                    //printf("%d\n", pool.size());
                    //for (int h = 0; h < pool.size(); h++)
                    //  {
                    //printf("bb[%d]\tee[%d]\n", pool[s1].pool->begin(), pool[s1].pool->end());

                    //We initialize hamming distance and iterate through the current pool and find the minimum distance.
                    hamming = 130;

                    
                    for(auto iter = pool.begin(); iter != pool.end(); ++iter)
                        {
                            auto c = iter->first;
                            unsigned char *cur = (unsigned char *)&c[0];
                            unsigned char *str = (unsigned char *)&s1[0];
                            
                            hamming = hamming_distance( str, cur );
                            
                            if(hamming <= bestHam) {
                                if(hamming == bestHam) { //add indexes to current best hamming distance
                                    memcpy(bestPool+ (bCount*16), cur, 16);
                                    bCount++;

                                } else { //new best hamming distance, add new index
                                    memcpy(nBestPool, bestPool, bCount*16);
                                    nbCount = bCount;
                                    bCount = 0;
                                    nBestHam = bestHam;
                                    bestHam = hamming;
                                    memcpy(bestPool + bCount*16, cur, 16);   
                                    bCount++;
                                }
                            } else if(hamming <= nBestHam) {
                                if(hamming == nBestHam) { //add indexes to next best hamming distance
                                    memcpy(nBestPool+(nbCount*16), cur, 16);
                                    nbCount++;
                                } else { //new next best hamming, add new index                                                                    
                                    nbCount = 0;
                                    memcpy(nBestPool, cur, 16);
                                    nbCount++;
                                    nBestHam = hamming;
                                }
                            }                                                                                                            
                        }
                    //  }
                    */
                /*
                } else {
                    lastCounter++;
                    int min[pool[s1].pool->size()];                  
                    unsigned char min_pattern[pool[s1].pool->size() * 16];
                    int hamming;


                    //for (int h = 0; h < pool[s1].pool->size(); h++)
                    //   {
                    //printf("b[%d]\te[%d]\n", pool[s1].pool->begin(), pool[s1].pool->end());
                            hamming = 130;
                            //printf("!!!!!!!!!!!!!!%d\n", pool[s1].pool->size());
                            for(auto iter = pool[s1].pool->begin(); iter != pool[s1].pool->end(); ++iter)
                                {                                   
                                    auto c = iter->first;
                                    unsigned char *cur = (unsigned char *)&c[0];
                                    unsigned char *str = (unsigned char *)&s2[0];

                                    //std::unordered_map<std::string, struct area> &t_pool = *pool[s1].pool;
                                    //printf("%d\t%d\n", t_pool[c].begin, t_pool[c].end);
                                    hamming = hamming_distance( str, cur );
                                                                       
                                    if(hamming <= bestHam) {
                                        if(hamming == bestHam) {
                                            memcpy(bestPool+(bCount*16), cur, 16);
                                            bCount++;
                                        } else {
                                            memcpy(nBestPool, bestPool, 16); //might be wrong
                                            //memcpy(nBestPool, bestPool, bCount*16); // is the size paramater here correct? PROBABLY memcpy(,,16)
                                            nbCount = bCount;
                                            bCount = 0;
                                            nBestHam = bestHam;
                                            bestHam = hamming;
                                            memcpy(bestPool + bCount*16, cur, 16);   
                                            bCount++;
                                        }
                                    } else if(hamming <= nBestHam) {
                                        if(hamming == nBestHam) {
                                            memcpy(nBestPool+(nbCount*16), cur, 16);
                                            nbCount++;
                                        } else {                                           
                                            nbCount = 0;
                                            memcpy(nBestPool, cur, 16);
                                            nbCount++;
                                            nBestHam = hamming;
                                        }
                                    }                                                                                                             
                                }
                            //   }
                            
                            
                }*/
                //**************************************************************
                // printf("Best: %d ham: %d\tnBest: %d  ham: %d\n", bCount, bestHam, nbCount, nBestHam);
                int iterationBB = 0;
                int iterationNB = 0;

                int bb[bCount+nbCount];
                int ee[bCount+nbCount];

                int order_counter = 0;

                iterationBB = bCount;
                for(int jj = 0; jj < bCount; jj++) {                        
                    std::string tt( (char *)bestPool+jj*16, 16 ) ;
                    if(level0) {
                        bb[order_counter] = pool[tt].begin;
                        ee[order_counter++] = pool[tt].end;
                    } else if(level1){
                        std::unordered_map<std::string, struct area> &t_pool = *pool[s1].pool;
                        bb[order_counter] = t_pool[tt].begin;
                        ee[order_counter++] = t_pool[tt].end;
                    } else if(level2){
                        bb[order_counter] = begin;
                        bb[order_counter++] = end;
                    }
                }

                //if(bCount <= 1)
                iterationNB = nbCount;
                for(int jj = 0; jj < nbCount; jj++) {                        
                    std::string tt( (char *)nBestPool+jj*16, 16 ) ;
                    if(level0) {
                        bb[order_counter] = pool[tt].begin;
                        ee[order_counter++] = pool[tt].end;
                    } else if(level1){
                        std::unordered_map<std::string, struct area> &t_pool = *pool[s1].pool;
                        bb[order_counter] = t_pool[tt].begin;
                        ee[order_counter++] = t_pool[tt].end;
                    } else if(level2) {
                        bb[order_counter] = begin;
                        ee[order_counter++] = end;
                    }
                }
                
                //printf("\n");
                for(int ii = 0; ii < iterationBB+iterationNB; ii++) {
                    //printf("%d***%d\n", bb[ii], ee[ii]);
                    //if(ee[ii]-bb[ii] == 1)
                        ee[ii]++;
                    for(int x = bb[ii]; x < ee[ii]; x++) {                    
                        const Scalar *rowPtr = (*memMapping).data();
                        rowPtr+=(desc[x].idx*COLS);
                        res = 0;
                        
                        for(int iter = 0; iter < COLS; iter+=4) {
                            diff0 = rowPtr[iter] * queryPtr[iter];
                            diff1 = rowPtr[iter+1] * queryPtr[iter+1];
                            diff2 = rowPtr[iter+2] * queryPtr[iter+2];
                            diff3 = rowPtr[iter+3] * queryPtr[iter+3];
                            res+=(diff0 + diff1 + diff2 + diff3);
                            }

                        if(res > best) {
                            nextBest = best;
                            best = res;
                            nextBestIndex = bestIndex;
                            bestIndex = desc[x].idx;

                            } else if(res > nextBest && desc[x].idx != bestIndex) {
                            nextBest = res;
                            nextBestIndex = desc[x].idx;
                        }        
                    }
                }

            } else {        
            /*
             * Works in all cases, but the pool size we search can often be too large
             * We can probably get this number smaller by using hamming distance further down to get more precise results
             *
             * We stop at next most significant byte, we need to explore third most and possible fourth most siginficant byte (torkils opinion)
             *
             *Most of the times we get here it is because we hit the pool with only zero's in the most significant bit. This pool consist of allmoust as 
             * many pools as end-begin for the zero pool. this means that it is not nessesary to go any deeper, as no two depts will be similar, and 
             *hamming distance in this level is sufficient to differentiate between the descriptors in the pool. we can simply use hamming distance instead of 
             * dot product to hopefully find the two best matches faster. (Thomas opinion)
             * We have to use hamming direct becuse there are many small
             */



            pool_counter++;
            //std::cout << "begin: " << begin << " end: " << end << std::endl;
            //int search_area = end-begin;
            
            for(int x = begin; x <= end; x++) {                    
                const Scalar *rowPtr = (*memMapping).data();
                rowPtr+=(desc[x].idx*COLS);
                res = 0;
                for(int iter = 0; iter < COLS; iter+=4) {
                    diff0 = rowPtr[iter] * queryPtr[iter];
                    diff1 = rowPtr[iter+1] * queryPtr[iter+1];
                    diff2 = rowPtr[iter+2] * queryPtr[iter+2];
                    diff3 = rowPtr[iter+3] * queryPtr[iter+3];
                    res+=(diff0 + diff1 + diff2 + diff3);
                }
                
                if(res > best) {
                    nextBest = best;
                    best = res;
                    nextBestIndex = bestIndex;
                    bestIndex = desc[x].idx;
                }
                else if(res > nextBest && res != best) {
                    nextBest = res;
                    nextBestIndex = desc[x].idx;
                }        
            }
        }

        // printf("P[%d]***H[%d]\n", pool_counter, hamming_counter);
        //printf("FIRST[%d]\tLAST[%d]\n", firstCounter, lastCounter);
        const Scalar *tmp1 = (*memMapping).data();
        tmp1+=(bestIndex*COLS);
        (*pvec_distances)[i*NN] = metric(queryPtr, tmp1, COLS);
        (*pvec_indices)[i*NN] = IndMatch(i, bestIndex);

        const Scalar *tmp2 = (*memMapping).data();
        tmp2+=(nextBestIndex*COLS);
        (*pvec_distances)[i*NN+1] = metric(queryPtr, tmp2, COLS);
        (*pvec_indices)[i*NN+1] = IndMatch(i, nextBestIndex);
    }
    
    std::cout << "cnt: " << cnt << std::endl;
    return true;
  };
    
private:
    typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> BaseMat;
    /// Use a memory mapping in order to avoid memory re-allocation
    std::unique_ptr< Eigen::Map<BaseMat> > memMapping;
};

}  // namespace matching
}  // namespace openMVG

#endif  // OPENMVG_MATCHING_ARRAYMATCHER_BRUTE_FORCE_H
