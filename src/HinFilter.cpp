#include <iostream>
#include "HinfFilter.h"
using namespace std;

/* Constructor: */
HinfFilter::HinfFilter(int _n,  int _m, int _sigma) {
	n = _n;
	m = _m;
  sigma = _sigma;
}

/* Set Fixed Matrix */
void HinfFilter::setFixed( MatrixXf _A, MatrixXf _C, MatrixXf _Q, MatrixXf _R ){
	A = _A;
	C = _C;
	Q = _Q;
	R = _R;
	I = I.Identity(n, n);
}

/* Set Fixed Matrix */
void HinfFilter::setFixed( MatrixXf _A, MatrixXf _B, MatrixXf _C, MatrixXf _Q, MatrixXf _R){
	A = _A;
	B = _B;
	C = _C;
	Q = _Q;
	R = _R;
	I = I.Identity(n, n);
}

/* Set Initial Matrix */
void HinfFilter::setInitial( VectorXf _X0, MatrixXf _P0 ){
	X0 = _X0;
	P0 = _P0;
}

/* Do prediction based of physical system (No external input)
*/	 
void HinfFilter::predict(void){
  X = (A * X0);
  P = (A * P0 * A.transpose()) + Q;
}

/* Do prediction based of physical system (with external input)
* U: Control vector
*/	 
void HinfFilter::predict( VectorXf U ){
  X = (A * X0) + (B * U);
  P = (A * P0 * A.transpose()) + Q;
}

/* Correct the prediction, using mesaurement 
*  Y: mesaure vector
*/
void HinfFilter::correct ( VectorXf Y ) {
  R_inv = R.inverse();
  P_old = P;
  innov_num = Y - C*X;
  norm_num = norm(innov_num);
  K = P_old * (I-theta*P_old+C.transpose()*R_inv*C*P_old).inverse()*C.tranpose()*R_inv;
  X = X + K *(innov_num);
  P = P_old * (I - theta*P_old + C.transpose() * R_inv * C * P_old).inverse();

  X0 = X;
  P0 = P;
}