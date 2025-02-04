/*
This file is part of the Ristra wonton project.
Please see the license file at the root of this repository, or at:
    https://github.com/laristra/wonton/blob/master/LICENSE
*/



#ifndef WONTON_LS_FITS_H_
#define WONTON_LS_FITS_H_

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "wonton/support/CoordinateSystem.h"
#include "wonton/support/Point.h"
#include "wonton/support/Matrix.h"
#include "wonton/support/svd.h"

namespace Wonton {


/*!
  @brief Compute least squares gradient from set of values
  @param[in] coords Vector of coordinates at which values are given
  @param[in] vals   Vector of values at said coordinates

  Compute a least squares gradient from a set of values. The first
  point is assumed to be the point where the gradient must be computed
  and the first value is assumed to the value at this reference point

  This operator does not know anything about a mesh.

*/

template<int D, typename CoordSys = CartesianCoordinates>
Vector<D> ls_gradient(std::vector<Point<D>> const & coords,
                      std::vector<double> const & vals) {

  CoordSys::template verify_coordinate_system<D>();

  Point<D> coord0 = coords[0];

  double val0 = vals[0];

  // There are nvals but the first is the reference point where we
  // are trying to compute the gradient; so the matrix sizes etc
  // will only be nvals-1

  int nvals = vals.size();

  // Each row of A contains the components of the vector from
  // coord0 to the candidate point being used in the Least Squares
  // approximation (X_i-X_0).

  Matrix A(nvals-1, D);
  for (int i = 0; i < nvals-1; ++i) {
    for (int j = 0; j < D; ++j)
      A[i][j] = coords[i+1][j]-coord0[j];
  }


  // A is a matrix of size nvals-1 by D (where D is the space
  // dimension). So transpose(A)*A is D by D

  Matrix AT = A.transpose();

  Matrix ATA = AT*A;

  // Each entry/row of F contains the difference between the
  // function value at the candidate point and the function value
  // at the point where we are computing (f-f_0)

  std::vector<double> F(nvals-1);
  for (int i = 0; i < nvals-1; ++i)
    F[i] = vals[i+1]-val0;

  // F is a vector of nvals. So transpose(A)*F is vector of D
  // (where D is the space dimension)

  Vector<D> ATF = Vector<D>(AT*F);

  // Inverse of ATA

  Matrix ATAinv = ATA.inverse();

  // Gradient of length D

  auto gradient = ATAinv*ATF;

  // Corrections for curvilinear coordinates

  CoordSys::modify_gradient(gradient, coord0);

  // Return

  return ATAinv*ATF;
}


/*!
  @brief Compute least squares quadfit from set of values
  @param[in] coords Vector of coordinates at which values are given
  @param[in] vals   Vector of values at said coordinates

  Compute a least squares quadfit from a set of values. The first
  point is assumed to be the point where the quadfit must be computed
  and the first value is assumed to the value at this reference point

  This operator does not know anything about a mesh.

*/

template<int D>
  // int N = D*(D+3)/2;  
  Vector<D*(D+3)/2> ls_quadfit(std::vector<Point<D>> const & coords,
			       std::vector<double> const & vals, 
			       bool const boundary_element) {

  Point<D> coord0 = coords[0];

  double val0 = vals[0];

  // There are nvals but the first is the reference point where we
  // are trying to compute the quadfit; so the matrix sizes etc
  // will only be nvals-1

  int nvals = vals.size();

  if (boundary_element) {
    // Not enough values to do a quadratic fit - drop down to linear fit
    
    Vector<D> grad = ls_gradient(coords, vals);
    Vector<D*(D+3)/2> result;
    for (int i = 0; i < D*(D+3)/2; i++) result[i] = 0.0;
    // Fill in the begining of the vector with gradient terms
    for (int i = 0; i < D; i++) result[i] = grad[i];
    return result;
  }


  // Each row of A contains the components of the vector from
  // coord0 to the candidate point being used in the Least Squares
  // approximation (X_i-X_0), up to quadratic order. Index j0
  // labels the columns up to D (e.g., gradient). Index j1
  // labels the D(D+1)/2 extra columns for the quadratic terms.
  // Index i labels rows for data points away from the center of
  // the point cloud (i.e., array[0]) with nvals points, contained 
  // in the array coords(nvals,D).

  Matrix A(nvals-1, D*(D+3)/2); // don't include 0th point
  for (int i = 0; i < nvals-1; i++) {
    int j1 = D; // index of colunns with quadrtic terms
    for (int j0 = 0; j0 < D; ++j0) {
      A[i][j0] = coords[i+1][j0]-coords[0][j0];
      // Add columns with the remaining quadratic delta terms
      // for each i, starting at D+j, reusing linear delta terms
      for (int k=0; k <= j0; k++) {
  	A[i][j1] = A[i][k]*A[i][j0];
  	j1 += 1;
      }
    }
  }

  size_t ma = D*(D+3)/2;
  std::vector<std::vector<double> > u(nvals-1, std::vector<double>(ma));
  std::vector<std::vector<double> > v(ma, std::vector<double>(ma));
  std::vector<double> w(ma);
  for (int i=0; i < nvals-1; i++) {
    for (int j=0; j < D*(D+3)/2; j++) {
      u[i][j] = A[i][j];
    }
  }
  
  int err = svd(u,w,v);

  // "edit" the singular values (eigenvalues):
  // (1) find wmax = largest value of w
  // (2) select only w's between TOL*wmax <= w <= wmax
  // -->Any other w's contribute 0 to the solution
  double scale = 1e-5;
  double wmax = 0.0;
  for (int j=0; j<ma; j++) {
    if (w[j] > wmax) wmax=w[j];
  }
  double thresh = scale*wmax;
  for (int j=0; j<ma; j++) {
   if (w[j] < thresh) w[j]=0.0;
  }
  
  // Each entry/row of F contains the difference between the
  // function value at the candidate point and the function value
  // at the point where we are computing (f-f_0)

  std::vector<double> F(nvals-1);
  for (int i = 0; i < nvals-1; ++i) {
    F[i] = vals[i+1]-val0;
  }

  // solve the problem for coefficients, in "result".
  std::vector<double> result(ma);
  svd_solve(u,w,v,F,result);

  return result;
}

}  // namespace Wonton

#endif
