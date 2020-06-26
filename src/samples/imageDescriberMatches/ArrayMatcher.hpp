//
// Created by Amir Yalamov (https://github.com/AmirYalamov)
// and Honson Tran (https://github.com/honsontran)
// on 2020-06-26.
//

#ifndef ALICEVISION_HNSWMATCHER_HPP
#define ALICEVISION_HNSWMATCHER_HPP

// add amy other necessary imports
#include <vector>


// import hnswlib
#include "src/dependencies/hnswlib/hnswlib/hnswlib.h"

using namespace hnswlib;

// create an ArrayMatcher class
class ArrayMatcher {

// methods accessible to everyone
public:

    bool Build (/* add necessary arguments, look into OpenMVG vs AV for argument types */) {

        // create matching structure
    }

    bool SearchNeighbour (/* add necessary arguments, look into OpenMVG vs AV for argument types */) {

        // perform search
    }
};

// @Honson, this should the last line in the file, write all code above this line
#endif // ALICEVISION_HNSWMATCHER_HPP
