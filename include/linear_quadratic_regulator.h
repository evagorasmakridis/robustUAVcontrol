//
//  lqr.h
//  control
//
//  Created by Evagoras Makridis on 13/03/2019.
//  Copyright © 2019 Evagoras Makridis. All rights reserved.
//

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <vector>
#include <assert.h>
#include <complex>
#include <cmath>
#include <cstdlib>
#pragma once

class LQR {
public:
    /* Problem Dimension */
    int n; //State vector dimension
    int j; //Control vector dimension
    /* Fixed Matrix */
    Eigen::MatrixXf A; //System dynamics matrix
    Eigen::MatrixXf B; //Control matrix
    Eigen::MatrixXf Q; //Process Noise Covariance matrix
    Eigen::MatrixXf R; //Measurement Noise Covariance matrix
    /* Variable Matrix */
    Eigen::MatrixXf S;
    Eigen::MatrixXf Klqr; //LQR Gain matrix
    
    /* Constructor */
    LQR(int _n,  int _j);

    void setFixed ( Eigen::MatrixXf _A, Eigen::MatrixXf _B, Eigen::MatrixXf _Q, Eigen::MatrixXf _R );
    void solver ( void );
};


/// Computes the unique stabilizing solution X to the discrete-time algebraic
/// Riccati equation:
///
/// \f[
/// A'XA - X - A'XB(B'XB+R)^{-1}B'XA + Q = 0
/// \f]
///
/// Based on the Schur Vector approach outlined in this paper:
/// "On the Numerical Solution of the Discrete-Time Algebraic Riccati Equation"
/// by Thrasyvoulos Pappas, Alan J. Laub, and Nils R. Sandell
///
Eigen::MatrixXf DiscreteAlgebraicRiccatiEquation(
    const Eigen::Ref<const Eigen::MatrixXf>& A,
    const Eigen::Ref<const Eigen::MatrixXf>& B,
    const Eigen::Ref<const Eigen::MatrixXf>& Q,
    const Eigen::Ref<const Eigen::MatrixXf>& R);


/// Returns true if and only if the two matrices are equal to within a certain
/// absolute elementwise @p tolerance.  Special values (infinities, NaN, etc.)
/// do not compare as equal elements.
template <typename DerivedA, typename DerivedB>
bool is_approx_equal_abstol(const Eigen::MatrixBase<DerivedA>& m1,
                            const Eigen::MatrixBase<DerivedB>& m2,
                            double tolerance) {
  return (
      (m1.rows() == m2.rows()) &&
      (m1.cols() == m2.cols()) &&
      ((m1 - m2).template lpNorm<Eigen::Infinity>() <= tolerance));
}

/// Returns true if and only if a simple greedy search reveals a permutation
/// of the columns of m2 to make the matrix equal to m1 to within a certain
/// absolute elementwise @p tolerance. E.g., there exists a P such that
/// <pre>
///    forall i,j,  |m1 - m2*P|_{i,j} <= tolerance
///    where P is a permutation matrix:
///       P(i,j)={0,1}, sum_i P(i,j)=1, sum_j P(i,j)=1.
/// </pre>
/// Note: Returns false for matrices of different sizes.
/// Note: The current implementation is O(n^2) in the number of columns.
/// Note: In marginal cases (with similar but not identical columns) this
/// algorithm can fail to find a permutation P even if it exists because it
/// accepts the first column match (m1(i),m2(j)) and removes m2(j) from the
/// pool. It is possible that other columns of m2 would also match m1(i) but
/// that m2(j) is the only match possible for a later column of m1.
template <typename DerivedA, typename DerivedB>
bool IsApproxEqualAbsTolWithPermutedColumns(
    const Eigen::MatrixBase<DerivedA>& m1,
    const Eigen::MatrixBase<DerivedB>& m2, double tolerance) {
  if ((m1.cols() != m2.cols()) || (m1.rows() != m2.rows())) return false;

  std::vector<bool> available(m2.cols());
  for (int i = 0; i < m2.cols(); i++) available[i] = true;

  for (int i = 0; i < m1.cols(); i++) {
    bool found_match = false;
    for (int j = 0; j < m2.cols(); j++) {
      if (available[j] &&
          is_approx_equal_abstol(m1.col(i), m2.col(j), tolerance)) {
        found_match = true;
        available[j] = false;
        break;
      }
    }
    if (!found_match) return false;
  }
  return true;
}
