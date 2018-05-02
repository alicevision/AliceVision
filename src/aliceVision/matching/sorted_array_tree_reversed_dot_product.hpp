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
    int idx;
    unsigned char *data;
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


void hamming_distance( unsigned char* A, unsigned char* B ) 
{
	unsigned int g[4];
	int sum = 0, i = 0;

	while ( sum < 32 ) 
	{
		g[0] = (unsigned int)*(A + (i * 16)) ^ (unsigned int)*(B + (i * 16));
		g[1] = (unsigned int)*(A + 4 + (i * 16)) ^ (unsigned int)*(B + 4 + (i * 16));
		g[2] = (unsigned int)*(A + 8 + (i * 16)) ^ (unsigned int)*(B + 8 + (i * 16));
		g[3] = (unsigned int)*(A + 12 + (i * 16)) ^ (unsigned int)*(B + 12 + (i * 16));

		std::cout << " g[0]: " << g[0] << " countSetBits: " << countSetBits(*g) << std::endl;
		std::cout << " g[1]: " << g[1] << " countSetBits: " << countSetBits(*(g + 1)) << std::endl;
		std::cout << " g[2]: " << g[2] << " countSetBits: " << countSetBits(*(g + 2)) << std::endl;
		std::cout << " g[3]: " << g[3] << " countSetBits: " << countSetBits(*(g + 3)) << std::endl;

		sum = countSetBits(*g) + countSetBits(*(g + 1)) + countSetBits(*(g + 2)) + countSetBits(*(g + 3));
		printf("sum: %d\n", sum);
		printf("\n");

		i++;
		if ( i == 8 ) break;
	}
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
            //int tmp = string_cmp(A[pos].data, B);
            if (tmp < 0) pos += inc;
            else if (tmp > 0) pos -= inc;
            else return pos;
            inc = inc/2;
        }

        return pos;
    }
    
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

    //create root
    //struct node *root = newNode( 1 );
    int i;
    struct node *root[8];

    for (i = 0; i < 8; i++)
        root[i] = newNode( 1 );
    
	//struct node *root = newNode( 1 );
    unsigned char A[128];
    unsigned char B[128]; 
    
    
    //std::vector<DistanceType> vec_distance((*memMapping).rows(), 0.0);

    //transpose all first.
    struct descriptor desc[(*memMapping).rows()];
    unsigned char *transposed_array = (unsigned char * )malloc((*memMapping).rows() * (*memMapping).cols());


    Scalar * row = (*memMapping).data();
    //transpose & set up tree
    //for (int queryIndex=0; queryIndex < nbQuery; ++queryIndex)
    for (i = 0; i < (*memMapping).rows(); ++i)
    {

        Scalar * test_row = (*memMapping).data();
        //       Scalar * queryPtr = mat_query.row(i).data();
        transpose( (unsigned char *)test_row+(i*(*memMapping).cols()), A );
        organize( A, B );
        memcpy( transposed_array + (i*(*memMapping).cols()), B, 128 );
        desc[i].idx = i;
        desc[i].data = transposed_array + (i*(*memMapping).cols());
        
        //insert_descriptor( root[0], B + 16, i );

    }

    qsort(desc, (*memMapping).rows(), sizeof(struct descriptor), cmp);


    //Scalar * row = (*memMapping).data();
    //for (i = 0; i < (*memMapping).rows(); ++i)
    for (int i=0; i < nbQuery; ++i)
        {

            const Scalar * queryPtr = mat_query.row(i).data();
            //transpose( (unsigned char *)row, A );
            transpose( (unsigned char *)queryPtr, A );
            organize( A, B );
            int pos = find_pos(desc, B, (*memMapping).rows(), (*memMapping).cols());
            //            printf("%d ", desc[pos].idx);
            
            
            //depth = binarySearch( root[0], B + 16 );

            int diff0, diff1, diff2, diff3, dota;
            int best = 0, nextBest = 0;

            int tmp, res;
            int bestIndex = -1;
            int nextBestIndex = -1;
            
            int AREA = 10;
          
            Scalar * test = (*memMapping).data();
            if(pos >= AREA/2 && pos <= (*memMapping).rows()-AREA/2) {                
                for(int x = pos-(AREA/2); x < pos+(AREA/2); x++) {                    
                    const Scalar *rowPtr = (*memMapping).data();
                    rowPtr+=(desc[x].idx*(*memMapping).cols());
                    res = 0;
                    for(int iter = 0; iter < (*memMapping).cols(); iter+=4) {
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
                    } else if(res > nextBest) {
                        nextBest = res;
                        nextBestIndex = desc[x].idx;
                    }
                    
                }

            } else if(pos >= AREA){
                for(int x = pos-AREA; x < pos; x++) {

                    const Scalar *rowPtr = (*memMapping).data();
                    rowPtr+=(desc[x].idx*(*memMapping).cols());
                    res = 0;
                    for(int iter = 0; iter < (*memMapping).cols(); iter+=4) {
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
                    } else if(res > nextBest) {
                        nextBest = res;
                        nextBestIndex = desc[x].idx;
                    }
                                       
                }

            } else {
                for(int x = pos; x < pos+AREA; x++) {

                    const Scalar *rowPtr = (*memMapping).data();
                    rowPtr+=(desc[x].idx*(*memMapping).cols());
                    res = 0;
                    for(int iter = 0; iter < (*memMapping).cols(); iter+=4) {
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
                    } else if(res > nextBest) {
                        nextBest = res;
                        nextBestIndex = desc[x].idx;
                    }                                     
                }
            }


            const Scalar *tmp1 = (*memMapping).data();
            tmp1+=(bestIndex*(*memMapping).cols());
            (*pvec_distances)[i*NN] = metric(queryPtr, tmp1, (*memMapping).cols());
            (*pvec_indices)[i*NN] = IndMatch(i, bestIndex);

            const Scalar *tmp2 = (*memMapping).data();
            tmp2+=(nextBestIndex*(*memMapping).cols());
            (*pvec_distances)[i*NN+1] = metric(queryPtr, tmp2, (*memMapping).cols());
            (*pvec_indices)[i*NN+1] = IndMatch(i, nextBestIndex);
            
            /* 
            const int maxMinFound = (int) std::min( size_t(NN), vec_distance.size());
            using namespace stl::indexed_sort;
            //std::vector< sort_index_packet_ascend< DistanceType, int> > packet_vec(vec_distance.size());
            std::vector< sort_index_packet_descend< DistanceType, int> > packet_vec(vec_distance.size());
            sort_index_helper(packet_vec, &vec_distance[0], maxMinFound);

            */
            /* 
               for (int j = 0; j < maxMinFound; ++j)
                {
                    //printf("%l\n", packet_vec[j].val);
                    (*pvec_distances)[i*NN+j] = packet_vec[j].val;
                    (*pvec_indices)[i*NN+j] = IndMatch(i, packet_vec[j].index);                    
                }
            
             for (int j = 8; j < 10; ++j)
                {
                    printf("%f\n", packet_vec[j].val);
                    (*pvec_distances)[i*NN+(j-8)] = packet_vec[j].val;
                    (*pvec_indices)[i*NN+(j-8)] = IndMatch(i, packet_vec[j].index);                    
                }
           
            */
            /*  
                int counter = 0;
                int place1 = desc[pos].idx;

            
                int place2 = 0;
                const Scalar * tmp1 = (*memMapping).data();

                tmp1+=place1*(*memMapping).cols();
                vec_distance[place1] = metric(queryPtr, tmp1, (*memMapping).cols());

                if(pos < (*memMapping).rows()-1)
                place2 = desc[pos+1].idx;
                else
                place2 = desc[pos-1].idx;

                const Scalar * tmp2 = (*memMapping).data();
                tmp2+=place2*(*memMapping).cols();
                vec_distance[place2] = metric(queryPtr, tmp2, (*memMapping).cols());
            
            
                (*pvec_distances)[i*NN] = vec_distance[place1];
                (*pvec_indices)[i*NN] = IndMatch(i, place1);

                (*pvec_distances)[i*NN+1] = vec_distance[place2];
                (*pvec_indices)[i*NN+1] = IndMatch(i, place2);
            */
            // row += (*memMapping).cols();
            
        }


    
    //std::cout << "cnt found depth msb: " << cnt << std::endl;
    /*
      #pragma omp parallel for schedule(dynamic)
      for (int queryIndex=0; queryIndex < nbQuery; ++queryIndex) 
      {
      std::vector<DistanceType> vec_distance((*memMapping).rows(), 0.0);
      const Scalar * queryPtr = mat_query.row(queryIndex).data();
      const Scalar * rowPtr = (*memMapping).data();
      for (int i = 0; i < (*memMapping).rows(); ++i)
      {
      vec_distance[i] = metric( queryPtr,
      rowPtr, (*memMapping).cols() );
      rowPtr += (*memMapping).cols();
      }

      // Find the N minimum distances:
      const int maxMinFound = (int) std::min( size_t(NN), vec_distance.size());
      using namespace stl::indexed_sort;
      std::vector< sort_index_packet_ascend< DistanceType, int> > packet_vec(vec_distance.size());
      sort_index_helper(packet_vec, &vec_distance[0], maxMinFound);

      for (int i = 0; i < maxMinFound; ++i)
      {
      (*pvec_distances)[queryIndex*NN+i] = packet_vec[i].val;
      (*pvec_indices)[queryIndex*NN+i] = IndMatch(queryIndex, packet_vec[i].index);
      }
      }
    */
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
