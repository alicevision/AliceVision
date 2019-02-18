#include "SolverIteration.hpp"

namespace aliceVision {
namespace intrinsicDecomposition {

void saveSolverResults(std::string directory, std::string suffix,
    const std::vector<SolverIteration>& optGNIters,
    const std::vector<SolverIteration>& optLMIters,
    bool optDoublePrecision)
{
    std::ofstream resultFile(directory + "results" + suffix + ".csv");
    resultFile << std::scientific;
    resultFile << std::setprecision(20);
    std::string colSuffix = optDoublePrecision ? " (double)" : " (float)";
	resultFile << "Opt(GN) Error" << colSuffix << ",  Opt(LM) Error";
	resultFile << "Opt(GN) Iter Time(ms)" << colSuffix << ", Opt(LM) Iter Time(ms)";
	resultFile << "Total Opt(GN) Time(ms)" << colSuffix << ", Total Opt(LM) Time(ms)" << colSuffix << std::endl;
    double sumOptGNTime = 0.0;
    double sumOptLMTime = 0.0;

    auto _optLMIters = optLMIters;
    auto _optGNIters = optGNIters;
    
    if (_optLMIters.size() == 0) {
        _optLMIters.push_back(SolverIteration(0, 0));
    }
    if (_optGNIters.size() == 0) {
        _optGNIters.push_back(SolverIteration(0, 0));
    }
    for (int i = 0; i < (int)std::max((int)_optLMIters.size(), (int)_optGNIters.size()); i++)
    {
        double optGNTime = ((_optGNIters.size() > i) ? _optGNIters[i].timeInMS : 0.0);
        double optLMTime = ((_optLMIters.size() > i) ? _optLMIters[i].timeInMS : 0.0);
        sumOptGNTime += optGNTime;
        sumOptLMTime += optLMTime;
        resultFile << i << ", " << clampedRead(_optGNIters, i).cost << ", " << clampedRead(_optLMIters, i).cost << ", " << optGNTime << ", " << optLMTime << ", " << sumOptGNTime << ", " << sumOptLMTime << std::endl;
    }
}


void reportFinalCosts(std::string name, const SolverParams& params, double gnCost, double lmCost)
{
    std::cout << "===" << name << "===" << std::endl;
    std::cout << "**Final Costs**" << std::endl;
    std::cout << "Opt GN,Opt LM" << std::endl;
    std::cout << std::scientific;
    std::cout << std::setprecision(20);
    if (params.useOptGN) {
        std::cout << gnCost;
    }
    std::cout << ",";
    if (params.useOptLM) {
        std::cout << lmCost;
    }
    std::cout << std::endl;
}

}
}
