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

struct node
{
	int data;
    int idx;
	struct node *left;
	struct node *right;
};

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
    
void printBits( unsigned char num ) 
{
  for ( int bit = 0; bit < 8; bit++ ) 
    {
      printf("%i", num & 0x01);
      num = num >> 1;
    }
}


void printFeature( unsigned char *c ) 
{
  for ( int i = 0; i < 8; i++ ) 
    {
      for (int j = 0; j < 16; j++)
	{
	  printBits(c[ ( i * 16 ) + j]);
	  printf( " " );
	  //printf("%d ", (int)(( i * 16 ) + j));
	}
      printf( "\n" );
		
    }

  printf( "\n\n" );
}


/* Function to get no of set bits in binary
representation of passed binary no. */
unsigned int countSetBits( unsigned int n )
{
	unsigned int count = 0;
	
	while ( n )
	{
		n &= ( n - 1 );
		count++;
	}
	
	return count;
}

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




/* newNode() allocates a new node with the given data and NULL left and
right pointers. */
struct node* newNode( int data )
{
	// Allocate memory for new node 
	struct node* node = (struct node*)malloc( sizeof(struct node) );

	// Assign data to this node
	node->data = data;

    //Asign count
    node->idx = 0;

	// Initialize left and right children as NULL
	node->left = NULL;
	node->right = NULL;
	return node;
}


int binarySearch( struct node *root, unsigned char * f ) {
    unsigned char num;
	struct node *tmp;
    unsigned int missmatch_count = 0; 
    unsigned int missmatch_roof = 5; //could be more, pass as parameter? 
    unsigned int cnt = 0;
	tmp = root;
	//std::cout << std::endl;

    for ( int i = 0; i < 16; i ++ ) {
        num = f[i];
        for ( int bit = 0; bit < 8; bit++ )  {
            if ( num & 0x01 )  {
                if ( tmp->right == NULL ) {
                    if (missmatch_count > missmatch_roof) {
                        break;
                    } else {
                        missmatch_count++;
                        tmp = tmp->left;
                        //std::cout << "(" << 0 << " missmatch! )";
                    }
                } else {
                    tmp = tmp->right;
                    //std::cout << 1;
                }
                
                cnt++;
            } else {
                if ( tmp->left == NULL ) {
                    if ( missmatch_count > missmatch_roof ) {
                        break;
                    } else {
                        missmatch_count++;
                        tmp = tmp->right;
                        //std::cout << "(" << 1 << " missmatch! )";
                    }
                } else {
                    tmp = tmp->left;
                    //std::cout << 0;
                }

                cnt++;
            }
            
            num = num >> 1;
        }
    }

    if (cnt == 128)
    return tmp->idx;
    else return -1;
}

//inserts only first dimention for now
    void insert_descriptor( struct node *root, unsigned char *f, int idx) 
{
	unsigned char num;
	struct node *tmp;
	tmp = root;

	for ( int i = 0; i < 16; i ++ ) 
	{
		num = f[i];
		for ( int bit = 0; bit < 8; bit++ ) 
		{
			if ( num & 0x01 ) 
			{
				if ( tmp->right == NULL ) 
					tmp->right = newNode( num );
				tmp = tmp->right;
				//std::cout << 1;
			} 
			else 
			{
				if ( tmp->left == NULL )
					tmp->left = newNode( num );
				tmp = tmp->left;
				//std::cout << 0;
			}

            num = num >> 1;
		}
	}

    tmp->idx = idx;

	//std::cout << std::endl;
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



    

    int find_pos(struct descriptor *A, unsigned char * B, int desc, int size)
    {
        int pos = desc/2;
        int inc = pos/2;
        int min = 0;
        while (pos > min && pos < desc && inc > 1)
        {
            int tmp = compareTo(A[pos].data, B);
            if (tmp < 0) pos += inc;
            else if (tmp > 0) pos -= inc;
            else return pos;            
            inc = inc/2;
        }
        //printf("%d\n", A[pos].change);
        return pos;
    }

    void shift_up(unsigned char * A) {
        
        for(int i = 0; i < 128; i++)
            A[i] += 16;
    }

    void shift_down(unsigned char * A) {

        for(int i = 0; i < 128; i++)
            A[i] -= 16;

    }

    bool in_list(unsigned char * pattern, unsigned char *list, int size) {
        for (int i = 0; i < size; i++) {
            if(memcmp(pattern, list + i * 16, size) == 0)
                return true;
        }

        return false;
    }

    
    /*
    void insert(struct area *a, unsigned char * data, int i) {
        std::string s( (char *) data, 16 ) ;

        
        if (a->pool.find(s) != a->pool.end())
            a->pool[s]->end = i;
             else {
            struct area b;
            b.begin = i;
            b.end = i;
            a->pool[s] = b;
            }
    }
*/
    
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
    unsigned char *transposed_array = (unsigned char * )malloc(ROWS * COLS);
    Scalar *tmp_row = (Scalar*)malloc(COLS);
    Scalar * row = (*memMapping).data();

    
    //transpose & set up tree
    for (i = 0; i < (*memMapping).rows(); ++i)
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

//std::unordered_map<std::string, struct area> &t_pool = *pool[s1].pool;
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
        int begin = 0, end = ROWS;
        if (pool.find(s1) != pool.end()) //if s1 exists in pool
        {
            if (pool[s1].pool->find(s2) != pool[s1].pool->end()) //if s2 exists in pool[s1].pool
            {

                //printf("EXACT MATCH\n");
                std::unordered_map<std::string, struct area> &t_pool = *pool[s1].pool;
		if (t_pool[s2].end - t_pool[s2].begin > 1)
		{
		    begin = t_pool[s2].begin;
		    end = t_pool[s2].end;
		    cnt++;
		}
		else
		{
		    if (pool[s1].end - pool[s1].begin > 1)
		    {
			begin = pool[s1].begin;
			end = pool[s1].end;
		    }
		}
            }
            else
            {
                //printf("NO\n");
		if (pool[s1].end - pool[s1].begin > 1)
                begin = pool[s1].begin;
                end = pool[s1].end;
            } 
        }
        
        int diff0, diff1, diff2, diff3, dota;
        int best = -1, nextBest = -1;
        int tmp, res;
        int bestIndex = -1;
        int nextBestIndex = -1;

          

	//std::cout << "begin: " << begin << " end: " << end << std::endl;
	for(int x = begin; x < end; x++) {                    
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
    
    // std::cout << "cnt: " << cnt << std::endl;
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
