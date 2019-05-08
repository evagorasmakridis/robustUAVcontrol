//
//  lqr.cpp
//  control
//
//  Created by Evagoras Makridis on 13/03/2019.
//  Copyright © 2019 Evagoras Makridis. All rights reserved.
//
#include <iostream>
#include "dji_sdk_demo/LinearQuadraticRegulator.h"
#include "dji_sdk_demo/discrete_algebraic_riccati_equation.h"

using namespace std;

/* Constructor: */
LQR::LQR(int _n,  int _j) {
    n = _n;
    j = _j;
}

/* Set Fixed Matrix */
void LQR::setFixed( MatrixXd _A, MatrixXd _B, MatrixXd _Q, MatrixXd _R ){
    A = _A;
    B = _B;
    Q = _Q;
    R = _R;
}

void LQR::solver( void ){
    S = DiscreteAlgebraicRiccatiEquation(A, B, Q, R);
    Eigen::MatrixXd tmp = B.transpose() * S * B + R;
    Klqr = tmp.llt().solve(B.transpose()*S*A);//(R + B.transpose()*S*B).inverse() + B.transpose()*S*A; //tmp.llt().solve(B.transpose()*S*A);
}
