#pragma once


namespace aliceVision {
namespace intrinsicDecomposition {

struct SolverParams
{
    bool useOptGN = false; // true;
    bool useOptLM = true; // false;
    bool earlyOut = false;
    unsigned int numIter = 1;
    unsigned int nonLinearIter = 3;
    unsigned int linearIter = 200;
    unsigned int patchIter = 32;
    bool profileSolve = true;
    bool optDoublePrecision = false;
};

}
}
