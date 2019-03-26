#include <iostream>
#include "MCCKalmanFilter.h"
using namespace std;

/* Constructor: */
MCCKalmanFilter::MCCKalmanFilter(int _n,  int _m, int _sigma) {
	n = _n;
	m = _m;
  sigma = _sigma;
}

/* Set Fixed Matrix */
void MCCKalmanFilter::setFixed( MatrixXf _A, MatrixXf _C, MatrixXf _Q, MatrixXf _R ){
	A = _A;
	C = _C;
	Q = _Q;
	R = _R;
	I = I.Identity(n, n);
}

/* Set Fixed Matrix */
void MCCKalmanFilter::setFixed( MatrixXf _A, MatrixXf _B, MatrixXf _C, MatrixXf _Q, MatrixXf _R){
	A = _A;
	B = _B;
	C = _C;
	Q = _Q;
	R = _R;
	I = I.Identity(n, n);
}

/* Set Initial Matrix */
void MCCKalmanFilter::setInitial( VectorXf _X0, MatrixXf _P0 ){
	X0 = _X0;
	P0 = _P0;
}

/* Do prediction based of physical system (No external input)
*/	 
void MCCKalmanFilter::predict(void){
  X = (A * X0);
  P = (A * P0 * A.transpose()) + Q;
}

/* Do prediction based of physical system (with external input)
* U: Control vector
*/	 
void MCCKalmanFilter::predict( VectorXf U ){
  X = (A * X0) + (B * U);
  P = (A * P0 * A.transpose()) + Q;
}

/* Correct the prediction, using mesaurement 
*  Y: mesaure vector
*/
void MCCKalmanFilter::correct ( VectorXf Y ) {
  R_inv = R.inverse();
  cov_P = P;
  innov_num = Y - C*X_old;
  innov_den = X_old - A*X;
  norm_num = norm(innov_num);
  norm_den = norm(innov_den);
  // norm_innov = sqrt((innov).'*invers_R*(innov));
  Gkernel = exp(-(pow(norm_num,2)) /(2*pow(sigma,2)))/exp(-(pow(norm_den,2))/(2*pow(sigma,2)));
  K = (cov_P.inverse() + Gkernel * C.transpose()*R_inv*C).inverse() * Gkernel * C.transpose()*R_inv;
  X = X + K *(innov_num);
  P = (I - K*C) * cov_P * (I - K*C).transpose() + (K*R*K.transpose());

  X0 = X;
  P0 = P;
}