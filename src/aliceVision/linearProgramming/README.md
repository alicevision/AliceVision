# linearProgramming

Linear programming [LP] is a technique for the optimization of a linear objective function, subject to linear equality and linear inequality constraints such as:

$$
\begin{align} & \text{maximize} && \mathbf{c}^\mathrm{T} \mathbf{x}\\
& \text{subject to} && A \mathbf{x} \leq \mathbf{b} \\
& \text{and} && \mathbf{x} \ge \mathbf{0} \end{align}
$$

where ``x`` represents the vector of variables (to be determined), ``c`` and ``b`` are vectors of (known) coefficients, ``A`` is a (known) matrix of coefficients.

## Linear programming tools

This project provides tools to:

- configure Linear programs (LP container),
- solve Linear Programs (convex or quasi convex ones).

### Linear program container

This project provides a generic container for LP (Linear Programming problems) that can be dense of sparse.

```cpp
// Dense LP
LPConstraints
// Sparse LP
LPConstraintsSparse
```

It allows to embed:

- objective function ``c`` and the problem type (minimization or maximization),
- constraints (coefficients ``A``, Sign, objective value ``b``),
- bounds over ``x`` parameters (<=, =, >=).

### Linear program solvers

This project provides access to different solvers (not exhaustive):

- OSI_CLP (COIN-OR) project,
- MOSEK commercial, free in a research context.

Those solver have been choosen due to the stability of their results and ability to handle large problems without numerical stability (LPSolve and GPLK have been discarded after extensive experiments).

See examples from linearProgramming_test.cpp.

### Linear programming module usage

The linear programming module of openMVG can be used for:

- solve classic linear problem (optimization),
- test the feasibility of linear problem,
- optimize upper bound of feasible problem (quasi-convex linear programs).

#### classic linear problem solving (optimization)

Here an example of usage of the framework:

```cpp
// Setup the LP (fill A,b,c and the constraint over x)
LPConstraints cstraint;
BuildLinearProblem(cstraint);

// Solve the LP with the solver of your choice
std::vector<double> vec_solution(2);
#if OPENMVG_HAVE_MOSEK  
MOSEKSolver solver(2);
#else
OSI_CISolverWrapper solver(2);
#endif
// Send constraint to the LP solver
solver.setup(cstraint);

// If LP have a solution
if (solver.solve())
// Get back estimated parameters
solver.getSolution(vec_solution);
```

#### Linear programming, feasible problem

This project can be use also to test only the feasibility of a given LP problem

$$
\begin{align} & \text{find} && \mathbf{x}\\
& \text{subject to} && A \mathbf{x} \leq \mathbf{b} \\
& \text{and} && \mathbf{x} \ge \mathbf{0} \end{align}
$$

#### Linear programming, quasi convex optimization

This project used a lot of L infinity minimisation formulation.
Often the posed problems are quasi-convex and dependent of an external parameter that we are looking for (i.e the maximal re-projection error for which the set of contraint is still feasible).

Optimization of this upper bound parameter can be done by iterating over all the possible value or by using a bisection that reduce the search range at each iteration.

```c+pp
// Require: gammaLow, gammUp (Low and upper bound of the parameter to optimize)
// Require: the LP problem (cstraintBuilder)
//Ensure: the optimal gamma value, or return infeasibility of the contraints set.

BisectionLP(ISolver & solver,
            ConstraintBuilder & cstraintBuilder,
            double gammaUp  = 1.0,  // Upper bound
            double gammaLow = 0.0,  // lower bound
            double eps      = 1e-8, // precision that stop dichotomy
            const int maxIteration = 20) // max number of iteration
{
    ConstraintType constraint;
    do
    {
        ++k; // One more iteration

        double gamma = (gammaLow + gammaUp) / 2.0;

        //-- Setup constraint and solver
        cstraintBuilder.Build(gamma, constraint);
        solver.setup( constraint );
        
        //-- Solving
        bool bFeasible = solver.solve();

        //-- According feasibility update the corresponding bound
        //-> Feasible, update the upper bound
        //-> Not feasible, update the lower bound
        (bFeasible) ? gammaUp = gamma; : gammaLow = gamma;
      
    } while (k < maxIteration && gammaUp - gammaLow > eps);
}
```

## Multiple View Geometry solvers based on L-Infinity minimization

This project provides Linear programming based solvers for various problem in computer vision by minimizing at the same time the maximal error over a series of cost function and some model parameters. It uses a L-Infinity minimization method.

This project implements problems introduced by [LinfNorm] and generalized by [LinfNormGeneric] to solve multiple view geometry problem.

Rather than considering quadratic constraints that require SOCP (Second Orde Cone Programming) we consider their LP (linear program) equivalent. It makes usage of residual error expressed with absolute error ( ``|a|<b``). Inequalities are transformed in two linear inequalities ``a<b`` and ``-b<-a`` to be used in the LP framework. Using LP rather than SCOP allow to have better solving time and easier constraint to express (see. [Arnak] for more explanation).

This project proposes solvers for the following problems:

- N-view triangulation [LinfNorm],
- Resection or pose matrix estimation [LinfNorm],
- Estimation of translations and structure from known rotations,

  - two formulations are implemented,

    - the simple one [LinfNorm],
    - the robust based on slack variables [OlssonDuality].

- Translation averaging:
  - Registration of relative translations to compute global translations [GlobalACSfM].