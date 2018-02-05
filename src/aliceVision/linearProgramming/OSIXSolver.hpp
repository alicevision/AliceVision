// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "OsiClpSolverInterface.hpp"
#include <aliceVision/config.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
#include "OsiMskSolverInterface.hpp"
#endif

#include "aliceVision/numeric/numeric.hpp"
#include "aliceVision/linearProgramming/ISolver.hpp"

#include "CoinPackedMatrix.hpp"
#include "CoinPackedVector.hpp"

#include <vector>

namespace aliceVision   {
namespace linearProgramming  {

/// OSI_X wrapper for the ISolver
template<typename SOLVERINTERFACE>
class OSIXSolver : public ISolver
{
public :
  OSIXSolver(int nbParams);

  ~OSIXSolver();

  //--
  // Inherited functions :
  //--

  bool setup(const LPConstraints & constraints);
  bool setup(const LPConstraintsSparse & constraints);

  bool solve();

  bool getSolution(std::vector<double> & estimatedParams);

private :
  SOLVERINTERFACE *si;
};


typedef OSIXSolver<OsiClpSolverInterface> OSI_CISolverWrapper;
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
typedef OSIXSolver<OsiMskSolverInterface> OSI_MOSEK_SolverWrapper;
#endif // ALICEVISION_HAVE_MOSEK



template<typename SOLVERINTERFACE>
OSIXSolver<SOLVERINTERFACE>::OSIXSolver(int nbParams) : ISolver(nbParams)
{
  si = new SOLVERINTERFACE;
  si->setLogLevel(0);
}

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_MOSEK)
template<>
OSIXSolver<OsiMskSolverInterface>::OSIXSolver(int nbParams) : ISolver(nbParams)
{
  si = new OsiMskSolverInterface();
  //si->setLogLevel(0);
}
#endif // ALICEVISION_HAVE_MOSEK

template<typename SOLVERINTERFACE>
OSIXSolver<SOLVERINTERFACE>::~OSIXSolver()
{
  // Memory cleaning.
  if ( si != nullptr )
  {
    delete si;
    si = nullptr;
  }
}

template<typename SOLVERINTERFACE>
bool OSIXSolver<SOLVERINTERFACE>::setup(const LPConstraints & cstraints) //cstraints <-> constraints
{
  bool bOk = true;
  if ( si == nullptr )
  {
    return false;
  }
  assert(_nbParams == cstraints._nbParams);

  const unsigned int NUMVAR = cstraints._constraintMat.cols();
  std::vector<double> col_lb(NUMVAR);//the column lower bounds
  std::vector<double> col_ub(NUMVAR);//the column upper bounds

  this->_nbParams = NUMVAR;

  si->setObjSense( ((cstraints._bminimize) ? 1 : -1) );

  const Mat & A = cstraints._constraintMat;

  //Equality constraint will be done by two constraints due to the API limitation ( >= & <=).
  const size_t nbLine = A.rows() +
    std::count(cstraints._vec_sign.begin(), cstraints._vec_sign.end(), LPConstraints::LP_EQUAL);

  std::vector<double> row_lb(nbLine);//the row lower bounds
  std::vector<double> row_ub(nbLine);//the row upper bounds

  CoinPackedMatrix * matrix = new CoinPackedMatrix(false,0,0);
  matrix->setDimensions(0, NUMVAR);

  //-- Add row-wise constraint
  size_t indexRow = 0;
  for (int i=0; i < A.rows(); ++i)
  {
    Vec temp = A.row(i);

    CoinPackedVector row;
    if ( cstraints._vec_sign[i] == LPConstraints::LP_EQUAL || cstraints._vec_sign[i] == LPConstraints::LP_LESS_OR_EQUAL )
    {
      int coef = 1;
      for ( int j = 0; j < A.cols() ; j++ )
      {
        row.insert(j, coef * temp.data()[j]);
      }
      row_lb[indexRow] = -1.0 * si->getInfinity();
      row_ub[indexRow] = coef * cstraints._Cst_objective(i);
      matrix->appendRow(row);
      indexRow++;
    }
    if ( cstraints._vec_sign[i] == LPConstraints::LP_EQUAL || cstraints._vec_sign[i] == LPConstraints::LP_GREATER_OR_EQUAL )
    {
      int coef = -1;
      for ( int j = 0; j < A.cols() ; j++ )
      {
	      row.insert(j, coef * temp.data()[j]);
      }
      row_lb[indexRow] = -1.0 * si->getInfinity();
      row_ub[indexRow] = coef * cstraints._Cst_objective(i);
      matrix->appendRow(row);
      indexRow++;
    }
  }

  //-- Setup bounds for all the parameters
  if (cstraints._vec_bounds.size() == 1)
  {
    // Setup the same bound for all the parameters
    for (int i=0; i < this->_nbParams; ++i)
    {
      col_lb[i] = cstraints._vec_bounds[0].first;
      col_ub[i] = cstraints._vec_bounds[0].second;
    }
  }
  else // each parameter have it's own bounds
  {
    for (int i=0; i < this->_nbParams; ++i)
    {
      col_lb[i] = cstraints._vec_bounds[i].first;
      col_ub[i] = cstraints._vec_bounds[i].second;
    }
  }

  si->loadProblem(*matrix, &col_lb[0], &col_ub[0], cstraints._vec_cost.empty() ? nullptr : &cstraints._vec_cost[0], &row_lb[0], &row_ub[0] );

  delete matrix;

  return bOk;
}

template<typename SOLVERINTERFACE>
bool OSIXSolver<SOLVERINTERFACE>::setup(const LPConstraintsSparse & cstraints) //cstraints <-> constraints
{
  bool bOk = true;
  if ( si == nullptr )
  {
    return false;
  }
  assert(_nbParams == cstraints._nbParams);

  const int NUMVAR = cstraints._constraintMat.cols();
  std::vector<double> col_lb(NUMVAR);//the column lower bounds
  std::vector<double> col_ub(NUMVAR);//the column upper bounds

  this->_nbParams = NUMVAR;

  si->setObjSense( ((cstraints._bminimize) ? 1 : -1) );

  const sRMat & A = cstraints._constraintMat;

  //Equality constraint will be done by two constraints due to the API limitation (>= & <=)
  const size_t nbLine = A.rows() +
    std::count(cstraints._vec_sign.begin(), cstraints._vec_sign.end(), LPConstraints::LP_EQUAL);

  std::vector<double> row_lb(nbLine);//the row lower bounds
  std::vector<double> row_ub(nbLine);//the row upper bounds

  CoinPackedMatrix * matrix = new CoinPackedMatrix(false,0,0);
  matrix->setDimensions(0, NUMVAR);

  //-- Add row-wise constraint
  size_t rowindex = 0;
  for (int i=0; i < A.rows(); ++i)
  {
    std::vector<int> vec_colno;
    std::vector<double> vec_value;
    for (sRMat::InnerIterator it(A,i); it; ++it)
    {
      vec_colno.push_back(it.col());
      vec_value.push_back(it.value());
    }


    if ( cstraints._vec_sign[i] == LPConstraints::LP_EQUAL || cstraints._vec_sign[i] == LPConstraints::LP_LESS_OR_EQUAL )
    {
      int coef = 1;
      row_lb[rowindex] = -1.0 * si->getInfinity();
      row_ub[rowindex] = coef * cstraints._Cst_objective(i);
      matrix->appendRow( vec_colno.size(),
	                 &vec_colno[0],
	                 &vec_value[0] );
      rowindex++;
    }

    if ( cstraints._vec_sign[i] == LPConstraints::LP_EQUAL || cstraints._vec_sign[i] == LPConstraints::LP_GREATER_OR_EQUAL )
    {
      int coef = -1;
      for ( std::vector<double>::iterator iter_val = vec_value.begin();
	      iter_val != vec_value.end();
	      iter_val++)
      {
  	    *iter_val *= coef;
      }
      row_lb[rowindex] = -1.0 * si->getInfinity();
      row_ub[rowindex] = coef * cstraints._Cst_objective(i);
      matrix->appendRow( vec_colno.size(),
	                 &vec_colno[0],
	                 &vec_value[0] );
      rowindex++;
    }
  }

  //-- Setup bounds for all the parameters
  if (cstraints._vec_bounds.size() == 1)
  {
    // Setup the same bound for all the parameters
    for (int i=0; i < this->_nbParams; ++i)
    {
      col_lb[i] = cstraints._vec_bounds[0].first;
      col_ub[i] = cstraints._vec_bounds[0].second;
    }
  }
  else  // each parameter have it's own bounds
  {
    for (int i=0; i < this->_nbParams; ++i)
    {
      col_lb[i] = cstraints._vec_bounds[i].first;
      col_ub[i] = cstraints._vec_bounds[i].second;
    }
  }

  si->loadProblem(
    *matrix,
    &col_lb[0],
    &col_ub[0],
    cstraints._vec_cost.empty() ? nullptr : &cstraints._vec_cost[0],
    &row_lb[0],
    &row_ub[0]);

  delete matrix;

  return bOk;
}

template<typename SOLVERINTERFACE>
bool OSIXSolver<SOLVERINTERFACE>::solve()
{
  //-- Compute solution
  if ( si != nullptr )
  {
    si->getModelPtr()->setPerturbation(50);
    si->initialSolve();
    return si->isProvenOptimal();
  }
  return false;
}

template<typename SOLVERINTERFACE>
bool OSIXSolver<SOLVERINTERFACE>::getSolution(std::vector<double> & estimatedParams)
{
  if ( si != nullptr )
  {
    const int n = si->getNumCols();
    memcpy(&estimatedParams[0], si->getColSolution(), n * sizeof(double));
    return true;
  }
  return false;
}

} // namespace linearProgramming
} // namespace aliceVision

