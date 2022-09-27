// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <vector>
#include <list>
#include <iostream>
#include <iterator>

namespace aliceVision {
namespace depthMap {

/* The LRUCache template takes a type T.
 * T must be copyable and offer operator<, operator==, as well as operator=(int)
 * and constructors T(), T(const T&) and T(int).
 * T is treated like an intrinsic type, but is not restricted to these types.
 *
 * Implement LRU caching functionality. This is meant to decouple the LRU-style use of
 * two pieces of cached information on the GPU:
 * - the dynamically allocated image pyramids
 * - the size of the GPU-constant camera parameters that is fixed at compile time
 * The LRUCache class may be more generally useful, for example for the host-sided
 * image cache.
 */
template <typename T>
class LRUCache
{
public:
    LRUCache( int size )
        : _owner( size, -1 )
        , _max_size( size )
    { }

    LRUCache( )
        : _max_size( 0 )
    { }

    void resize( int size )
    {
        _max_size = size;
        _owner.resize( size, -1 );
    }

    inline int getIndex( const T& val )
    {
        typename std::vector<T>::iterator o_it = std::find( _owner.begin(), _owner.end(), val );
        if( o_it != _owner.end() )
            return ( o_it - _owner.begin() );
        else
            return -1;
    }

    /* Insert the value val into a zero-based index set.
     * If the value is already in the set, the function returns false,
     *    position contains the value's index and oldVal is -1.
     * If the value is newly inserted and the index was not full before,
     *    the function returns true, position contains the value's index
     *    and oldVal is -1.
     * If the value replaces an entry of the index set, the function
     *    returns true, position contains the value's index and
     *    oldVal contains the value that was removed from the index set.
     * The replacement strategy is LRU.
     */
    inline bool insert( const T& val, int& position, T& oldVal )
    {
        if( _max_size == 0 )
        {
            std::stringstream ss;
            ss << __FILE__ << ":" << __LINE__ << " ERROR: LRUCache max size is 0, so we cannot insert any element." << std::endl;
            throw std::runtime_error(ss.str());
        }

        int cell;
        typename std::vector<T>::iterator o_it;
        
        o_it = std::find( _owner.begin(), _owner.end(), val );
        if( o_it != _owner.end() )
        {
            oldVal = -1;
            typename std::list<T>::iterator c_it = std::find( _cache.begin(), _cache.end(), val );
            _cache.erase( c_it );
            _cache.push_back( val );
            position = ( o_it - _owner.begin() );
            return false;
        }
        else if( ( cell = _cache.size() ) < _max_size )
        {
            oldVal = -1;
            _cache.push_back( val );
            _owner[cell] = val;
            position = cell;
            return true;
        }
        else
        {
            oldVal = _cache.front();
            _cache.pop_front();
            _cache.push_back( val );

            o_it = std::find( _owner.begin(), _owner.end(), oldVal );
            *o_it = val;
            position = ( o_it - _owner.begin() );
            return true;
        }
    }

    inline bool insert( const T& val, int* position )
    {
        T dummy;
        return insert( val, *position, dummy );
    }

    inline bool insert( const T& val, int* position, T* oldVal )
    {
        return insert( val, *position, *oldVal );
    }

    inline void clear()
    {
        _cache.clear();
        _owner.assign( _max_size, -1 );
    }

    inline std::ostream& dump( std::ostream& ostr )
    {
        typename std::ostream_iterator<T> out_it( ostr ," ");
        std::copy( _cache.begin(), _cache.end(), out_it );
        return ostr;
    }

private:
    std::list<T>   _cache;
    std::vector<T> _owner;
    int            _max_size;
};

} // namespace depthMap
} // namespace aliceVision

/*
 * a little test program:
 *
 * #include <iostream>
 * #include <vector>
 * #include <algorithm>
 * #include "lrucache.hpp"
 * 
 * using namespace aliceVision::depthMap;
 * 
 * int main()
 * {
 *     LRUCache<int> cache(10);
 *     int oldValue, position;
 * 
 *     std::vector<int> v(100);
 *     generate( v.begin(), v.end(), [](){ return (int)(rand()%20); } );
 * 
 *     for( auto newValue : v ) {
 *         if( cache.insert( newValue, position, oldValue ) ) {
 *             cout << newValue << " inserted at " << position;
 *             if( oldValue != -1 ) cout << " removed oldValue " << oldValue;
 *             std::cout << std::endl;
 *         } else
 *             std::cout << newValue << " cached at " << position << std::endl;
 *         cache.dump(cout) << endl;
 *     }
 * }
 */
