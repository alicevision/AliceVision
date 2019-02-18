#pragma once

#include "SolverParams.hpp"

#include <limits>
#include <vector>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <string>


namespace aliceVision {
namespace intrinsicDecomposition {

struct SolverIteration
{
    SolverIteration() {}
    SolverIteration(double _cost, double _timeInMS) { cost = _cost; timeInMS = _timeInMS; }
    double cost = -std::numeric_limits<double>::infinity();
    double timeInMS = -std::numeric_limits<double>::infinity();
};

template<class T>
const T& clampedRead(const std::vector<T> &v, int index)
{
    if (index < 0) return v[0];
    if (index >= v.size()) return v[v.size() - 1];
    return v[index];
}

void saveSolverResults(std::string directory, std::string suffix,
    const std::vector<SolverIteration>& optGNIters,
    const std::vector<SolverIteration>& optLMIters,
    bool optDoublePrecision);

void reportFinalCosts(std::string name, const SolverParams& params, double gnCost, double lmCost);

}
}
