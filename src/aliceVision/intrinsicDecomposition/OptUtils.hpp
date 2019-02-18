#pragma once
extern "C" {
#include "Opt.h"
}
#include "SolverIteration.hpp"
#include "NamedParameters.hpp"
#include "cudaUtil.hpp"

#include <vector>
#include <cmath>

#ifdef _WIN32
#include <Windows.h>
#endif

namespace aliceVision {
namespace intrinsicDecomposition {

#ifdef _WIN32
class SimpleTimer {
public:
    void init() {
        // get ticks per second
        QueryPerformanceFrequency(&frequency);

        // start timer
        QueryPerformanceCounter(&lastTick);
    }
    // Time since last tick in ms
    double tick() {
        LARGE_INTEGER currentTick;
        QueryPerformanceCounter(&currentTick);

        // compute and print the elapsed time in millisec
        double elapsedTime = (currentTick.QuadPart - lastTick.QuadPart) * 1000.0 / frequency.QuadPart;
        lastTick = currentTick;
        return elapsedTime;
    }
protected:
    LARGE_INTEGER frequency;
    LARGE_INTEGER lastTick;
};
#else
class SimpleTimer {
public:
    void init() {}
    // Time since last tick in ms
    double tick() { return nanf("");}
};
#endif



inline void launchProfiledSolve(Opt_State* state, Opt_Plan* plan, void** problemParams, std::vector<SolverIteration>& iterationSummary) {
    SimpleTimer t;
    t.init();
    Opt_ProblemInit(state, plan, problemParams);
    cudaDeviceSynchronize();
    double timeMS = t.tick();
    double cost = Opt_ProblemCurrentCost(state, plan);
    iterationSummary.push_back(SolverIteration(cost, timeMS));

    t.tick();
    while (Opt_ProblemStep(state, plan, problemParams)) {
        cudaDeviceSynchronize();
        timeMS = t.tick();
        cost = Opt_ProblemCurrentCost(state, plan);
        iterationSummary.push_back(SolverIteration(cost, timeMS));
        t.tick();
    }
}


template<class T> size_t index_of(T element, const std::vector<T>& v) {
    auto location = std::find(v.begin(), v.end(), element);
    if (location != v.end()) {
        return std::distance(v.begin(), location);
    }
    else {
        return (size_t)-1;
    }
}

template<class T> T* getTypedParameterImage(std::string name, const NamedParameters& solverParameters) {
    auto i = index_of(name, solverParameters.names());
    return (T*)(solverParameters.data()[i]);
}

// TODO: Error handling
template<class T> void findAndCopyArrayToCPU(std::string name, std::vector<T>& cpuBuffer, const NamedParameters& solverParameters) {
    auto i = index_of(name, solverParameters.names());
    cudaSafeCall(cudaMemcpy(cpuBuffer.data(), solverParameters.data()[i], sizeof(T)*cpuBuffer.size(), cudaMemcpyDeviceToHost));
}
template<class T> void findAndCopyToArrayFromCPU(std::string name, std::vector<T>& cpuBuffer, const NamedParameters& solverParameters) {
    auto i = index_of(name, solverParameters.names());
    cudaSafeCall(cudaMemcpy(solverParameters.data()[i], cpuBuffer.data(), sizeof(T)*cpuBuffer.size(), cudaMemcpyHostToDevice));
}
template<class T> T getTypedParameter(std::string name, const NamedParameters& params) {
    auto i = index_of(name, params.names());
    return *(T*)params.data()[i];
}

template<class T> void getTypedParameterIfPresent(std::string name, const NamedParameters& params, T& value) {
    auto i = index_of(name, params.names());
    if (i != (size_t)-1) {
        value = *(T*)params.data()[i];
    }
}


inline void setAllSolverParameters(Opt_State* state, Opt_Plan* plan, const NamedParameters& params) {
    for (auto param : params.getVector()) {
        Opt_SetSolverParameter(state, plan, param.name.c_str(), param.ptr);
    }
}

}
}
