#pragma once

#include "OptSolver.hpp"
#include "SolverParams.hpp"
#include "SolverIteration.hpp"

namespace aliceVision {
namespace intrinsicDecomposition {

/** We want to run several solvers in an identical manner, with some initalization
and finish code for each of the examples. The structure is the same for every
example, so we keep it in solveAll(), and let individual examples override
combinedSolveInit(); combinedSolveFinalize(); preSingleSolve(); postSingleSolve();*/
class CombinedSolverBase {
public:
    virtual void combinedSolveInit() = 0;
    virtual void combinedSolveFinalize() = 0;
    virtual void preSingleSolve() = 0;
    virtual void postSingleSolve() = 0;
    virtual void preNonlinearSolve(int iteration) = 0;
    virtual void postNonlinearSolve(int iteration) = 0;

    virtual void solveAll() {
        combinedSolveInit();
        for (auto& s : m_solverInfo) {
            if (s.enabled) {
                singleSolve(s);
            }
        }
        combinedSolveFinalize();
    }

    double getCost(std::string name) {
        for (auto s : m_solverInfo) {
            if (s.name == name) {
                if (s.solver && s.enabled) {
                    return s.solver->finalCost();
                }
            }
        }
        return nan("");
    }

    void setParameters(const SolverParams& params) {
        m_solverParams = params;
    }

    std::vector<SolverIteration> getIterationInfo(std::string name) {
        for (auto& s : m_solverInfo) {
            if (s.name == name) {
                if (s.solver && s.enabled) {
                    return s.iterationInfo;
                }
            }
        }
        return std::vector<SolverIteration>();
    }

    void addSolver(std::shared_ptr<SolverBase> solver, std::string name, bool enabled = true) {
        m_solverInfo.resize(m_solverInfo.size() + 1);
        m_solverInfo[m_solverInfo.size() - 1].set(solver, name, enabled);

    }

    void addOptSolvers(std::vector<unsigned int> dims, const std::string& problemFilename, bool doublePrecision = false) {
        if (m_solverParams.useOptGN) {
            addSolver(std::make_shared<OptSolver>(dims, problemFilename, "gaussNewtonGPU", doublePrecision), "Opt(GN)", true);
        }
        if (m_solverParams.useOptLM) {
            addSolver(std::make_shared<OptSolver>(dims, problemFilename, "LMGPU", doublePrecision), "Opt(LM)", true);
        }
    }

protected:
    struct SolverInfo
    {
        std::shared_ptr<SolverBase> solver;
        std::vector<SolverIteration> iterationInfo;
        std::string name;
        bool enabled;
        void set(std::shared_ptr<SolverBase>& _solver, std::string _name, bool _enabled)
        {
            solver = _solver;
            name = _name;
            enabled = _enabled;
        }
    };
    std::vector<SolverInfo> m_solverInfo;

    virtual void singleSolve(SolverInfo& s)
    {
        preSingleSolve();
        for (int i = 0; i < (int)m_solverParams.numIter; ++i)
        {
            std::cout << "//////////// ITERATION " << i+1 << "/" << m_solverParams.numIter << " -- " << s.name << " ///////////////" << std::endl;
            preNonlinearSolve(i);
            s.solver->solve(m_solverNamedParams, m_problemParams, m_solverParams.profileSolve, s.iterationInfo);
            postNonlinearSolve(i);
            if (m_solverParams.earlyOut || m_endSolveEarly)
            {
                m_endSolveEarly = false;
                break;
            }
        }
        postSingleSolve();
    }
    // Set to true in preNonlinearSolve or postNonlinearSolve to finish the solve before the specified number of iterations
    bool m_endSolveEarly = false;
    NamedParameters m_solverNamedParams;
    NamedParameters m_problemParams;
    SolverParams m_solverParams;
};

}
}
