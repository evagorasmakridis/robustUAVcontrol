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
#include <assert.h>
#include <complex>

using namespace Eigen;

class LQR {
    
public:
    /* Problem Dimension */
    int n; //State vector dimension
    int j; //Control vector dimension
    
    /* Fixed Matrix */
    MatrixXd A; //System dynamics matrix
    MatrixXd B; //Control matrix
    MatrixXd Q; //Process Noise Covariance matrix
    MatrixXd R; //Measurement Noise Covariance matrix
    
    /* Variable Matrix */
    MatrixXd S;
    MatrixXd Klqr; //LQR Gain matrix
    
    /* Constructor */
    LQR(int _n,  int _j);
    
    void setFixed ( MatrixXd _A, MatrixXd _B, MatrixXd _Q, MatrixXd _R );

    void solver ( void );
    
};
